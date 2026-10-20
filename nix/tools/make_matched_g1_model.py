"""Write an mc_mujoco G1 model matched to the mjlab training model (ArmWrenchRamp task).
Steps (cumulative, chosen with --parts): feet, joints, masses. Timestep 2.5 ms implicitfast always."""
import sys, argparse, os, numpy as np, mujoco
p=argparse.ArgumentParser(); p.add_argument("src"); p.add_argument("out"); p.add_argument("--parts", default="feet,joints,masses"); a=p.parse_args()
parts=set(a.parts.split(",")) if a.parts else set()
import mjlab.tasks
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
cfg=load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-RHP7Rewards-TrackGated-ExtAcc-ArmWrenchRamp", play=True); cfg.scene.num_envs=1
A=ManagerBasedRlEnv(cfg=cfg, device="cuda:0").sim.mj_model
nm=lambda m,t,i: (mujoco.mj_id2name(m,t,i) or "").split('/')[-1]
spec=mujoco.MjSpec.from_file(a.src)
spec.meshdir=os.path.join(os.path.dirname(os.path.abspath(a.src)), "../meshes")
spec.option.timestep=0.0025; spec.option.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
aj={nm(A,mujoco.mjtObj.mjOBJ_JOINT,i):i for i in range(A.njnt)}
ab={nm(A,mujoco.mjtObj.mjOBJ_BODY,i):i for i in range(A.nbody)}
if "joints" in parts: parts |= {"armature", "friction"}
for j in spec.joints:
  if j.name in aj and j.type==mujoco.mjtJoint.mjJNT_HINGE:
    d=A.jnt_dofadr[aj[j.name]]
    if "armature" in parts: j.armature=float(A.dof_armature[d])
    if "friction" in parts: j.damping=np.zeros(np.shape(j.damping)); j.frictionloss=0.0
if "masses" in parts:
  for b in spec.bodies:
    if b.name in ab and b.name not in ("world",):
      i=ab[b.name]; b.mass=float(A.body_mass[i]); b.ipos=A.body_ipos[i].copy(); b.iquat=A.body_iquat[i].copy(); b.inertia=A.body_inertia[i].copy(); b.explicitinertial=True
if "feet" in parts:
  for side in ("left","right"):
    b=spec.body(f"{side}_ankle_roll_link"); bi=ab[f"{side}_ankle_roll_link"]
    for g in list(b.geoms):
      if g.contype or g.conaffinity:
        if g.type==mujoco.mjtGeom.mjGEOM_SPHERE: spec.delete(g)
    for gi in range(A.ngeom):
      if A.geom_bodyid[gi]==bi and (A.geom_contype[gi] or A.geom_conaffinity[gi]):
        g=b.add_geom(); g.type=mujoco.mjtGeom(A.geom_type[gi]); g.size=A.geom_size[gi].copy(); g.pos=A.geom_pos[gi].copy(); g.quat=A.geom_quat[gi].copy()
        g.friction=A.geom_friction[gi].copy(); g.condim=int(A.geom_condim[gi]); g.solref=A.geom_solref[gi].copy(); g.solimp=A.geom_solimp[gi].copy()
        g.contype=int(A.geom_contype[gi]); g.conaffinity=int(A.geom_conaffinity[gi]); g.group=3; g.margin=float(A.geom_margin[gi])
m=spec.compile(); xml=spec.to_xml()
# mc_mujoco parses the XML itself and only knows <motor>; MjSpec writes motors as <general> (same dynamics).
xml=xml.replace("<general ", "<motor ")
open(a.out,"w").write(xml); print("wrote", a.out, "mass", m.body_mass.sum().round(3), "ngeom", m.ngeom, "parts", sorted(parts))
