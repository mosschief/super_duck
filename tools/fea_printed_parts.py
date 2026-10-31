#!/usr/bin/env python3
"""
Linear static FEA of the printed leg parts under the sim's worst load cases (cad/fea_loads.json, from
tools/export_fea_loads.py), before printing.

Each part (cad/freecad/printed/<part>.step, its body frame, mm) is meshed with gmsh (linear tets, ~2.5 mm) and solved
with quadratic (P2) elements in scikit-fem: clamped at its interface towards the trunk, the case's wrench spread over
its loaded interface as a rigid spider (RBE3: equal force share + the moment as a rotation field). Bolt circles are
modelled as annuli (the clamped area under the bolt heads / motor face), so the stress right at a hole edge is not
resolved: treat the hot spots there as approximate. Six unit load cases are solved once per part and every case is
their superposition.

Material: printed PETG modelled as a homogeneous solid, E = 1800 MPa, nu = 0.38 (walls + 45 % infill: the walls
carry most of the bending, the solid model reads somewhat stiffer and lower in stress than the print). Allowable
25 MPa (PETG across layers, the weak direction, whatever the print orientation); target safety factor >= 2 on the
99th-percentile element stress (the peak sits on the idealized constraint edges).

Run with the FEA venv (gmsh, scikit-fem, pyamg): ~/.venvs/fea/bin/python tools/fea_printed_parts.py [-k thigh]
Writes cad/fea_report.md (+ cad/fea_results.json).
"""
import argparse
import json
import math
import pathlib
import time

import gmsh
import numpy as np
import pyamg
from skfem import Basis, BilinearForm, ElementTetP2, ElementVector, MeshTet, asm, condense
from skfem.helpers import ddot, sym_grad, trace, eye

ROOT = pathlib.Path(__file__).resolve().parents[1]
PRINTED = ROOT / "cad" / "freecad" / "printed"
E, NU = 1800.0, 0.38                 # MPa
ALLOW = 25.0                         # MPa, PETG across layers
SF_TARGET = 2.0
LAM, MU = E * NU / ((1 + NU) * (1 - 2 * NU)), E / (2 * (1 + NU))


def annulus(centre, axis, r_mid, half_width, lo, hi):
    """Points within half_width of radius r_mid about the axis through centre, axial coordinate in [lo, hi]."""
    c, a = np.array(centre, float), np.array(axis, float)

    def sel(x):
        d = x - c[:, None]
        ax = a @ d
        r = np.linalg.norm(d - np.outer(a, ax), axis=0)
        return (np.abs(r - r_mid) <= half_width) & (ax >= lo) & (ax <= hi)
    return sel


def union(*sels):
    return lambda x: np.logical_or.reduce([s(x) for s in sels])


def rim_patch(x):
    """Wheel hub rim outer band (R > 66), the whole circumference (the patch is picked per case)."""
    return np.hypot(x[0], x[2]) > 66.0


def disc(centre, axis, r_max, lo, hi):
    """A clamped face: points within r_max of the axis, axial coordinate in [lo, hi]."""
    return annulus(centre, axis, r_max / 2, r_max / 2, lo, hi)


def box_sel(lo, hi):
    return lambda x: np.all((x >= np.array(lo)[:, None]) & (x <= np.array(hi)[:, None]), axis=0)


# Interfaces in each part's body frame (cad/freecad/build_super_duck.py). Held: the bolts (hole radius + 2 mm,
# through the thickness) and the face clamped against the motor / trunk (bonded). Loaded: the motor housing's bolt
# circle and its seating face.
PARTS = {
    "L_hip_mount": dict(files=["L_hip_mount"],
                        fixed=union(*[annulus((x, -54.0, 0), (0, 0, 1), 0.0, 4.25, -60, -35) for x in (-50, 0, 55)],
                                    box_sel((-58, -70, -43.4), (62, -20.75, -42.4))),      # shelf top on the floor
                        loaded=union(annulus((0, 24.0, -44.0), (1, 0, 0), 23.75, 3.35, -17.5, -10.5),
                                     annulus((0, 24.0, -44.0), (1, 0, 0), 23.5, 3.0, -11.5, -10.5))),
    "L_hip_roll_yoke": dict(files=["L_hip_roll_yoke"],
                            fixed=union(annulus((0, 0, 0), (1, 0, 0), 13.5, 3.7, 16.5, 22.5),
                                        disc((0, 0, 0), (1, 0, 0), 18.5, 16.5, 17.5)),       # GIM4305 output face
                            loaded=union(annulus((0, 0, 0), (0, 1, 0), 37.0, 3.7, 36.0, 41.0),
                                         annulus((0, 0, 0), (0, 1, 0), 36.0, 4.0, 40.0, 41.0))),
    "L_thigh": dict(files=["L_thigh_outer", "L_thigh_inner"],
                    fixed=union(annulus((0, 0, 0), (0, 1, 0), 12.5, 3.7, 16.0, 29.0),
                                disc((0, 0, 0), (0, 1, 0), 25.5, 22.5, 23.5)),               # GIM6010 output hub
                    loaded=union(annulus((0, 0, -118.0), (0, 1, 0), 37.0, 3.7, -20.0, 29.0),
                                 annulus((0, 0, -118.0), (0, 1, 0), 38.5, 5.5, 10.5, 11.5))),  # housing face y 11
    "L_shin": dict(files=["L_shin"],
                   fixed=union(annulus((0, 0, 0), (0, 1, 0), 12.5, 3.7, -36.0, -22.0),
                               disc((0, 0, 0), (0, 1, 0), 23.5, -23.0, -22.0)),               # GIM6010 output face
                   loaded=union(annulus((0, 0, -118.0), (0, 1, 0), 23.75, 3.35, -18.5, -12.5),
                                annulus((0, 0, -118.0), (0, 1, 0), 24.0, 4.0, -13.0, -12.5))),
    "L_wheel_hub": dict(files=["L_wheel_hub"],
                        fixed=union(annulus((0, 0, 0), (0, 1, 0), 13.5, 3.7, -3.5, 2.5),
                                    disc((0, 0, 0), (0, 1, 0), 18.5, 1.5, 2.5)),             # GIM4305 output face
                        loaded=rim_patch),
}


def mesh_part(files, size):
    for algo3d, heal in ((10, False), (10, True), (1, True), (4, True)):   # HXT, then on the healed shape
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("part")
        if heal:   # repair tiny / degenerate features (e.g. a counterbore breaking through a face edge)
            for opt in ("OCCFixDegenerated", "OCCFixSmallEdges", "OCCFixSmallFaces", "OCCSewFaces"):
                gmsh.option.setNumber(f"Geometry.{opt}", 1)
            gmsh.option.setNumber("Geometry.Tolerance", 0.05)
        tags = []
        for f in files:
            tags += gmsh.model.occ.importShapes(str(PRINTED / f"{f}.step"))
        if heal:
            gmsh.model.occ.healShapes()
        vols = gmsh.model.occ.getEntities(3)
        if len(vols) > 1:
            gmsh.model.occ.fuse(vols[:1], vols[1:])
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", size / 3)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
        gmsh.option.setNumber("Mesh.Algorithm3D", algo3d)
        try:
            gmsh.model.mesh.generate(3)
            if 4 not in gmsh.model.mesh.getElementTypes(3):
                raise RuntimeError("no tetrahedra")
            break
        except Exception as e:
            print(f"  mesh: algorithm {algo3d} failed ({str(e).splitlines()[0]}), retrying")
            gmsh.finalize()
    else:
        for deflection in (0.05, 0.15, 0.3):
            try:
                mesh_from_stl(files, size, deflection)
                break
            except Exception as e:
                print(f"  mesh: STL at {deflection} mm failed ({str(e).splitlines()[0]})")
                gmsh.finalize()
        else:
            raise RuntimeError("could not mesh " + ", ".join(files))
    ntags, coords, _ = gmsh.model.mesh.getNodes()
    xyz = coords.reshape(-1, 3)
    idx = {t: i for i, t in enumerate(ntags)}
    etypes, _, enodes = gmsh.model.mesh.getElements(3)
    tets = np.array([idx[n] for n in enodes[list(etypes).index(4)]]).reshape(-1, 4)
    gmsh.finalize()
    used = np.unique(tets)
    remap = -np.ones(len(xyz), int)
    remap[used] = np.arange(len(used))
    return xyz[used], remap[tets]


def mesh_from_stl(files, size, deflection):
    """Fallback when gmsh can't mesh the STEP (degenerate edges): FreeCAD tessellates the part finely, gmsh fills the
    closed surface with tets. Leaves the gmsh model meshed (caller reads it, then finalizes)."""
    import subprocess
    import tempfile
    tmp = pathlib.Path(tempfile.mkdtemp())
    stl = tmp / "part.stl"
    script = tmp / "tess.py"
    script.write_text("import Part, Mesh, MeshPart\n"
                      f"shapes = [Part.read(p) for p in {[str(PRINTED / (n + '.step')) for n in files]!r}]\n"
                      "s = shapes[0].fuse(shapes[1:]) if len(shapes) > 1 else shapes[0]\n"
                      f"m = MeshPart.meshFromShape(Shape=s, LinearDeflection={deflection}, AngularDeflection=0.2, Relative=False)\n"
                      f"m.write({str(stl)!r})\n")
    subprocess.run(["freecadcmd", str(script)], check=True, capture_output=True)
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.merge(str(stl))
    gmsh.model.mesh.classifySurfaces(math.radians(30), True, True, math.radians(180))
    gmsh.model.mesh.createGeometry()
    gmsh.model.geo.addVolume([gmsh.model.geo.addSurfaceLoop([t for _, t in gmsh.model.getEntities(2)])])
    gmsh.model.geo.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", size)
    gmsh.option.setNumber("Mesh.MeshSizeMin", size / 3)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.model.mesh.generate(3)
    print("  mesh: from FreeCAD STL (the STEP has degenerate edges)")


def spider(points, ref, F, M):
    """Nodal forces on `points` (n x 3) carrying force F and moment M (about ref) as a rigid spider."""
    c = points.mean(axis=0)
    r = points - c
    Mc = M + np.cross(ref - c, F)
    Ip = np.einsum("ij,ij->", r, r) * np.eye(3) - r.T @ r
    w = np.linalg.solve(Ip, Mc)
    return F / len(points) + np.cross(w, r)


def stress(basis, u):
    uh = basis.interpolate(u)
    eps = sym_grad(uh)
    return 2 * MU * eps + LAM * trace(eps) * eye(np.ones(eps.shape[2:]), 3)   # (3, 3, nel, nqp)


def von_mises(s):
    dev = s - np.einsum("ii...->...", s) / 3.0 * np.eye(3)[:, :, None, None]
    return np.sqrt(1.5 * np.einsum("ij...,ij...->...", dev, dev))


def analyze(name, spec, cases, size):
    t0 = time.time()
    xyz, tets = mesh_part(spec["files"], size)
    mesh = MeshTet(xyz.T, tets.T)
    basis = Basis(mesh, ElementVector(ElementTetP2()), intorder=2)

    @BilinearForm
    def elasticity(u, v, w):
        e_u, e_v = sym_grad(u), sym_grad(v)
        return 2 * MU * ddot(e_u, e_v) + LAM * trace(e_u) * trace(e_v)
    K = asm(elasticity, basis)
    locs = basis.doflocs                                    # (3, ndofs); dofs come in x, y, z triples per point
    comp = np.arange(locs.shape[1]) % 3
    pts_all = locs[:, comp == 0].T                          # one row per point
    fixed_pts = spec["fixed"](pts_all.T)
    loaded_mask = spec["loaded"](pts_all.T)
    D = np.nonzero(np.repeat(fixed_pts, 3))[0]
    ref = np.mean([c["at_mm"] for c in cases], axis=0)
    # rigid body modes for the AMG near-nullspace
    B = np.zeros((locs.shape[1], 6))
    for k in range(3):
        B[comp == k, k] = 1.0
    x, y, z = locs
    B[comp == 1, 3], B[comp == 2, 3] = -z[comp == 1], y[comp == 2]
    B[comp == 0, 4], B[comp == 2, 4] = z[comp == 0], -x[comp == 2]
    B[comp == 0, 5], B[comp == 1, 5] = -y[comp == 0], x[comp == 1]
    Kc, _, _, I = condense(K, np.zeros(K.shape[0]), D=D)
    ml = pyamg.smoothed_aggregation_solver(Kc.tocsr(), B=B[I], max_coarse=500)

    def solve(f):
        u = np.zeros(K.shape[0])
        u[I] = ml.solve(f[I], tol=1e-8, accel="cg", maxiter=400)
        return u

    def load_vector(mask, F, M, point):
        f = np.zeros(K.shape[0])
        nf = spider(pts_all[mask], point, F, M)
        sel = np.nonzero(mask)[0]
        for k in range(3):
            f[3 * sel + k] = nf[:, k]
        return f

    results = []
    if name == "L_wheel_hub":   # each case loads its own rim patch: solve them one by one
        unit = None
    else:
        unit = []
        for k in range(6):
            F, M = np.zeros(3), np.zeros(3)
            (F if k < 3 else M)[k % 3] = 1.0
            u = solve(load_vector(loaded_mask, F, M, ref))
            unit.append((u, stress(basis, u)))
    for c in cases:
        F, M = np.array(c["F_N"]), np.array(c["M_Nm"]) * 1000.0     # N*m -> N*mm
        if unit is not None:
            M_ref = M + np.cross(np.array(c["at_mm"]) - ref, F)
            coef = np.concatenate([F, M_ref])
            u = sum(a * uu for a, (uu, _) in zip(coef, unit))
            s = sum(a * ss for a, (_, ss) in zip(coef, unit))
        else:
            d = np.array(c["at_mm"]) / max(np.linalg.norm(c["at_mm"]), 1e-9)
            ang = np.degrees(np.arccos(np.clip((pts_all @ d) / np.maximum(np.linalg.norm(pts_all, axis=1), 1e-9), -1, 1)))
            mask = loaded_mask & (ang < 20.0)
            u = solve(load_vector(mask, F, np.zeros(3), np.array(c["at_mm"])))
            s = stress(basis, u)
        vm = von_mises(s).max(axis=1)                           # per element
        disp = np.linalg.norm(u.reshape(-1, 3)[loaded_mask], axis=1).max()
        worst = int(vm.argmax())
        results.append({**c, "vm_peak_MPa": float(vm.max()), "vm_p99_MPa": float(np.percentile(vm, 99)),
                        "peak_at_mm": [round(float(v), 1) for v in mesh.p[:, mesh.t[:, worst]].mean(axis=1)],
                        "disp_mm": float(disp)})
    info = dict(elements=int(mesh.t.shape[1]), dofs=int(K.shape[0]), fixed_points=int(fixed_pts.sum()),
                loaded_points=int(loaded_mask.sum()), seconds=round(time.time() - t0, 1))
    return results, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-k", default="")
    ap.add_argument("--size", type=float, default=2.5, help="mesh size (mm)")
    ap.add_argument("--printed", help="read the STEP files from this directory and only print the results "
                                      "(trying out a change; the report and results file are left alone)")
    args = ap.parse_args()
    global PRINTED
    if args.printed:
        PRINTED = pathlib.Path(args.printed).resolve()
    loads = json.loads((ROOT / "cad" / "fea_loads.json").read_text())["cases"]
    out, L = {}, ["# FEA of the Printed Leg Parts", "",
                  "Generated by `tools/fea_printed_parts.py` (gmsh + scikit-fem, P2 tets) from the load cases in "
                  "`fea_loads.json` (`tools/export_fea_loads.py`: the sim's worst instants, falls excluded). "
                  f"PETG as a solid, E {E:.0f} MPa; allowable {ALLOW:.0f} MPa (across layers); target safety factor "
                  f"{SF_TARGET:g} on the 99th-percentile element stress. Bolt circles are annuli, so hole-edge peaks "
                  "are approximate.", "",
                  "| Part | Worst case | von Mises p99 / peak (MPa) | Safety factor (p99) | Peak at (mm, part frame) | Interface deflection (mm) | Mesh |",
                  "| :--- | :--- | ---: | ---: | :--- | ---: | :--- |"]
    saved = ROOT / "cad" / "fea_results.json"
    out = json.loads(saved.read_text()) if saved.exists() else {}
    for name, spec in PARTS.items():
        if args.k not in name:
            continue
        try:
            res, info = analyze(name, spec, loads[name], args.size)
        except Exception as e:
            print(f"{name}: FAILED ({e})", flush=True)
            continue
        if args.printed:
            res.sort(key=lambda r: -r["vm_p99_MPa"])
            for c in res[:3]:
                print(f"{name}: {c['scenario']} ({c['kind']}): p99 {c['vm_p99_MPa']:.1f} MPa (SF "
                      f"{ALLOW / c['vm_p99_MPa']:.2f}), peak {c['vm_peak_MPa']:.1f} at {c['peak_at_mm']}, "
                      f"deflection {c['disp_mm']:.2f} mm", flush=True)
            continue
        out[name] = {"info": info, "cases": res}
        saved.write_text(json.dumps(out, indent=1) + "\n")
        print(f"{name}: done ({info['seconds']} s)", flush=True)
    if args.printed:
        return
    for name, r in out.items():
        res, info = r["cases"], r["info"]
        res.sort(key=lambda r: -r["vm_p99_MPa"])
        w = res[0]
        sf = ALLOW / w["vm_p99_MPa"]
        L.append(f"| `{name}` | {w['scenario']} ({w['kind']}, t {w['t']} s) | {w['vm_p99_MPa']:.1f} / {w['vm_peak_MPa']:.1f} | "
                 f"**{sf:.1f}**{'' if sf >= SF_TARGET else ' ✗'} | {', '.join(f'{v:g}' for v in w['peak_at_mm'])} | "
                 f"{w['disp_mm']:.2f} | {info['elements']} tets, {info['dofs']} dofs |")
        print(L[-1], flush=True)
    L += ["", "## Top cases per part", ""]
    for name, r in out.items():
        L += [f"### `{name}`", "", "| Case | F (N, part frame) | M (N·m) | p99 / peak (MPa) | Deflection (mm) |",
              "| :--- | :--- | :--- | ---: | ---: |"]
        for c in r["cases"][:5]:
            L.append(f"| {c['scenario']} ({c['kind']}) | {', '.join(f'{v:.0f}' for v in c['F_N'])} | "
                     f"{', '.join(f'{v:.2f}' for v in c['M_Nm'])} | {c['vm_p99_MPa']:.1f} / {c['vm_peak_MPa']:.1f} | {c['disp_mm']:.2f} |")
        L.append("")
    (ROOT / "cad" / "fea_report.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
