# Building and running on rl-tower (mc-rtc-nix)

`flake.nix` builds the mc_rtc stack pinned to Raphaël's 2026-08-13 workspace
(`../deploy/pinned-commits.txt`): mc_rtc, tvm, Tasks and state-observation from
bastien-muraccioli's forks, mc_g1 and g1_description from the
`topic/raphael-live-2026-08-13` branches (mathieu-celerier). ROS and Python are off,
so the g1_description URDF gets its `package://` URIs rewritten to absolute paths.

`build-workspace.sh` builds the rest from source into `$G1_WS/install`
(`$G1_WS` defaults to `~/devel/mc_rtc_nix_ws`, sources in `$G1_WS/src` at the pinned commits):
mc_joystick_plugin, mc_external_forces_observer (+ `deploy/mc_external_forces_observer-local.patch`),
mc_mujoco (bastien fork + Raphaël's change, `mathieu-celerier/mc_mujoco topic/raphael-live-2026-08-13`,
downloads MuJoCo 3.3.6 to `~/.mujoco`), g1_mj_description (Noceo200 5a013e0) and this controller.

```sh
nix develop ./nix -c ./nix/build-workspace.sh             # everything
nix print-dev-env ./nix > $G1_WS/run/env.sh                # avoid re-evaluating the flake per run
source $G1_WS/run/env.sh
mc_mujoco -f $G1_WS/run/mc_rtc.yaml --without-visualization --without-mc-rtc-gui
```

- `mc_rtc.rl-tower.yaml`: global config used as `run/mc_rtc.yaml` (module paths; set
  `RobotModulePaths` to the mc-g1 store path, `nix eval --raw ./nix#packages.x86_64-linux.mc-g1.outPath`).
- `G1RLQPController.user.yaml`: copy to `~/.config/mc_rtc/controllers/G1RLQPController.yaml`
  (policy paths on this machine; `acceleration_policy_index: 2` selects ArmWrenchRamp model_14999).

## Sim-to-sim matching (2026-10-20)

- `g1_mj_description`'s `g1_29dof.xml` runs at 1 ms RK4, so mc_mujoco uses `Frameskip = round(2.5/1) = 3`:
  the physics advances 3 ms per 2.5 ms control step. Use 2.5 ms implicitfast (training's) instead.
- `tools/make_matched_g1_model.py SRC OUT --parts feet,joints,masses` (run in the mjlab venv) writes a
  copy matched to the mjlab training model (timestep always; mjlab foot capsules/friction 0.6, per-joint
  armature, no damping/frictionloss, mjlab masses/inertias). `--parts ""` = timestep only.
  Select it with `mc_mujoco.g1_29dof.user.yaml` → `~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml`.
- `tools/run_case.sh NAME [extra controller yaml]` runs headless and prints speed, yaw drift, fall time
  and how often the QP overrides the ankle-pitch reference; `tools/acc_diag.py` compares policy q̈,
  QP output (`alphaDOut`, `ffOut_acceleration`), measured q̈ and the zero-base-torque base acceleration.
- `joint_limit_cbf: {zeta, lambda, ds_percent}` in the controller config overrides the Acc-CBF gains.
