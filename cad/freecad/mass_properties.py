"""Mass, CoM and full inertia of each trunk / leg body from super_duck.FCStd, in the MJCF body frames (mm, g).

Printed PETG at its walls + infill, purchased parts at their real masses, plus screw and wiring allowances.
Writes cad/freecad/mass_properties.json; the gim6010 MJCF's trunk and leg inertials come from it.
Usage: freecadcmd cad/freecad/mass_properties.py   (after build_super_duck.py)
"""
import os
import FreeCAD as App, json, sys
HERE = os.path.dirname(os.path.abspath(__file__))
from FreeCAD import Vector as V
d=App.openDocument(os.path.join(HERE, 'super_duck.FCStd'))
RHO=1.27  # PETG g/cm3
STRUCT=('hip_mount','yoke','thigh_outer','thigh_inner','_shin','wheel_hub')
SHELL=('trunk_tub','trunk_cover','trunk_lid')
FIXED={'battery_6s2p_cells':828,'battery_6s2p_wrap':42,'bms_6s_100a':60,'main_fuse_40a':10,'antispark_switch_120a':60,
 'buck_7v4_10a_servos':7,'buck_5v1_5a_pi':7,'buck_5v_5a_peripherals':7,'estop_22mm':25,'charge_port_gx16':15,
 'fan_40x10_5v':9,'raspberry_pi_5':46,'pi_active_cooler':25,'hat_standoffs_16mm':0.1,'ai_hat_plus_26tops':41,
 'teensy_carrier_pcb':5,'teensy_40':3,'can_transceivers':4,'teensy_terminals':4,'imu_icm42688p':2,'bus_servo_adapter':10,
 'max98357a_i2s_amp':2,'speaker_40mm_3w':27.3,'usb_nav_cam':8,'sts3215_neck_pitch':55}
def mass_of(lab, sh):
    v=sh.Volume/1000.0; a=sh.Area/100.0
    if lab in FIXED: return FIXED[lab],'fixed'
    if 'gim6010' in lab: return 388,'motor'
    if 'gim4305' in lab: return 150,'motor'
    if 'tire' in lab: return 90,'tire'
    if 'clamp_ring' in lab: return v*2.70,'Al'
    if 'grommet' in lab: return v*1.2,'rubber'
    if 'harness' in lab: return v*0.85,'harness'
    if any(k in lab for k in SHELL): t,f=0.126,0.18
    elif any(k in lab for k in STRUCT): t,f=0.252,0.45
    else: t,f=0.168,0.30
    w=min(v,a*t); return RHO*(w+(v-w)*f),'printed'
SCREWS={'Trunk':30,'L_HipMount':10,'L_HipRoll':10,'L_Thigh':10,'L_Shin':8,'L_Wheel':6}
SCREWS.update({k.replace('L_','R_'):v for k,v in SCREWS.items() if k.startswith('L_')})
WIRING={'Trunk':(120,V(-26,0,-10))}
res={}
for p in d.Objects:
    if p.TypeId!='App::Part' or p.Label not in SCREWS: continue
    items=[]
    for o in p.Group:
        if o.TypeId!='Part::Feature' or o.Shape.isNull(): continue
        sols=o.Shape.Solids
        if not sols: print('nosolid',o.Label); continue
        vol=sum(s.Volume for s in sols)
        m,kind=mass_of(o.Label,o.Shape)
        for s in sols:
            k=m*s.Volume/vol
            items.append((k, s.CenterOfMass, s.MatrixOfInertia, s.Volume, o.Label, kind))
    M=sum(i[0] for i in items)
    extra=[(SCREWS[p.Label],None)]
    if p.Label in WIRING: extra.append(WIRING[p.Label])
    c=V(0,0,0)
    for k,cm,_,_,_,_ in items: c+=cm*k
    c0=c*(1.0/M)
    ext=[(m_, pt if pt is not None else c0) for m_,pt in extra]
    Mt=M+sum(e[0] for e in ext)
    c=c+sum((pt*m_ for m_,pt in ext),V(0,0,0)); c=c*(1.0/Mt)
    I=[[0]*3 for _ in range(3)]
    def add(m,cm,Mi,vol):
        r=cm-c; rr=[r.x,r.y,r.z]
        for a in range(3):
            for b in range(3):
                base=(Mi.A[a*4+b]*(m/vol) if Mi is not None else 0.0)
                I[a][b]+=base+m*((r.Length**2 if a==b else 0)-rr[a]*rr[b])
    for k,cm,Mi,vol,_,_ in items: add(k,cm,Mi,vol)
    for m_,pt in ext: add(m_,pt,None,1)
    by={}
    for k,_,_,_,lab,kind in items: by[lab]=by.get(lab,0)+k
    res[p.Label]=dict(mass_g=Mt,com_mm=[c.x,c.y,c.z],I_gmm2=I,parts={k:round(v,1) for k,v in by.items()},extra=[e[0] for e in ext])
json.dump(res, open(os.path.join(HERE, 'mass_properties.json'), 'w'), indent=1)
for b,r in res.items(): print('RES',b,round(r['mass_g'],1),[round(x,1) for x in r['com_mm']])
