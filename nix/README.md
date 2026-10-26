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
**plus the FloatingBase angular-velocity fix, branch `topic/floatingbase-angvel-world` (64b1e89)**,
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
- `sim_test:` block (see `G1RLQPController.h`): timed velocity-command schedule and random horizontal
  pushes through mc_mujoco's `<robot>::ApplyForcesOnBody`. `tools/run_test.sh NAME tests/<file>.yaml [wall s]`
  runs one and prints per-segment tracking, pushes and falls; `tools/fall_rate.py *_test.csv` pools seeds.
- **The logged `FloatingBase_orientation` is world-to-body (sva convention)**: conjugate it before
  extracting yaw (the scripts here do).

## FloatingBase angular velocity frame (2026-10-21)

mc_mujoco filled the `FloatingBase` body sensor with MuJoCo's free-joint `qvel[3:6]`/`qacc[3:6]`,
which are body-frame, while mc_rtc's `BodySensorObserver` hands them to `Robot::velW`/`accW` as
world-frame (the linear parts are world-frame in both). The real robot's roll/pitch rates were thus
rotated by the base yaw: correct at the initial heading, swapped at ±90°. Every acceleration-policy
fall through the QP (turns, the stock model's yaw drift, hard pushes) started at |heading| ≈ 90–100°,
whatever the turn rate, direction, action law or sim model. Fixed in mc_mujoco
`topic/floatingbase-angvel-world` (upstream rohanpsingh/main has the same code).

- `sim_test.mjlab_action_law: true` sends MJLab's `JointAccelerationAction` torques
  (H [q̈_base; q̈_ref] + C − τ_ext filtered at `mjlab_tau_ext_cutoff_hz`, 5 Hz) instead of the QP's;
  `G1RLQPController_tauMjlab` / `_tauQP` are logged either way (they agree to ~20 % RMS, corr 0.95–0.99).
- `tools/make_matched_g1_model.py` extra parts: `priority` (foot friction 0.6 wins over the floor),
  `collision` (mjlab capsules + exclusions), `limits` (hip roll 139 Nm), `solver` (ls_iterations 20).
- `tools/queue_fix.sh`: the post-fix battery for both acceleration policies.

After the fix (`run/tests_fix.txt`), both ArmWrenchRamp `model_14999` and ArmWrenchJointDR `model_17998`:

| Test | 14999 | 17998 |
| --- | --- | --- |
| stock 1 ms model, straight 0.5 | 568 s, no fall (was 25 s) | 568 s, no fall (was 274 s) |
| stock 2.5 ms model | 1306 s, no fall | 1298 s, no fall |
| matched model, turn schedule (±0.5 rad/s, on the spot, walking) | no fall, wz ±0.46–0.48 | no fall, wz ±0.49–0.52 |
| matched, full command schedule | 834 s, no fall | 824 s, no fall |
| matched, 150 N pelvis pushes (3 seeds) | 0 falls in 2975 s | 0 falls in 2970 s |
| matched, 300 N pushes at the pelvis origin (3 seeds) | 3 falls, 2.4/min | 3 falls, 1.5/min |
| matched, 300 N pushes at the pelvis CoM (6 seeds, `tools/queue_push_com.sh`) | 6 falls, 0.81/min (mjlab 0.56) | 5 falls, 0.31/min (mjlab 0.38) |

No QP failures outside the 300 N falls. The origin-vs-CoM difference is the test, not the controller: `ApplyForcesOnBody` applies the force at the given body-frame point (default origin), 7.6 cm above the pelvis CoM, adding ~0.076·F of pitch/roll torque; mjlab's force push uses `xfrc_applied` at the CoM. `sim_test.pushes.at_com` (default `true`) now matches mjlab.

## IMU-based base estimation (2026-10-21)

The hardware pipeline of the Acc-CBF-QP paper (Muraccioli et al., IROS 2026, H1): `Encoder` → `Tilt`
(mc_state_observation, built with `-DWITH_ROS_OBSERVERS=OFF`, installs to `install/lib/mc_observers`) →
`ExternalForcesObserver`. The Tilt observer estimates base tilt and linear velocity from the torso IMU and a
contact anchor; the controller registers `G1RLQPController::anchorFrame` (sole centres weighted by |τ̂ext|
at the knees, no foot force sensors, as on the H1).

- `nix/tilt_pipeline.yaml`: the `ObserverPipelines` block, appended to the controller override
  (`run/ctl_override_jdr_tilt.yaml`). `G1RLQPController_realBasePosW/VelW` log the estimate next to the
  simulator's `FloatingBase_*`.
- mc_mujoco fills a body sensor `S` only from MuJoCo sensors named `S_gyro` / `S_accelerometer`; the
  `g1_mj_description` models have none for mc_g1's `Accelerometer`, so it was empty. `make_matched_g1_model.py
  --parts ...,imu` adds them on `torso_link` at mc_g1's pose (`g1_29dof_mjlab_imu.xml`). MuJoCo does not apply
  the XML `noise`: the IMU is perfect here.
- `model_17998`, matched model, estimated state (`run/tests_tilt.txt`, `tools/queue_tilt.sh`): turn schedule and
  full command schedule no fall; 150 N pushes 0 falls in 2792 s; 300 N at the CoM 6 falls in 601 s, 0.60/min
  (ground truth 0.31, mjlab 0.38). Body-frame linear-velocity error 0.008 m/s RMS walking straight, 0.04–0.06
  around pushes with 0.4–0.6 m/s peaks; tilt error 0.3°.

### IMU noise (2026-10-21)

mc_mujoco `topic/imu-noise` (624c665, on top of the angular-velocity fix) adds a robot-config key
`imuNoise: {gyro, accelerometer, gyroBias, accelerometerBias, seed}` (per-axis white-noise std, std of a constant
per-axis bias drawn once per sensor). `tools/queue_noise.sh`, `run/tests_noise.txt`, `model_17998`, Tilt pipeline:

| IMU | turn schedule, command schedule | 300 N at CoM, 6 seeds | estimate error, straight walk |
| --- | --- | --- | --- |
| perfect | no fall | 0.60 falls/min | tilt 0.06°, velocity 0.008 m/s RMS |
| n1: gyro 0.005 rad/s, acc 0.03 m/s², bias std 0.005 / 0.05 | no fall | 0.43 falls/min | same as perfect |
| n4: 4× n1 | no fall | 0.65 falls/min | tilt bias 3.4°, forward velocity −0.06 m/s, gyro bias 0.03 rad/s passed to the observation |

(Ground-truth base state 0.31, mjlab 0.38 falls/min; 5–6 falls per row, so ±40 %.)

## Raphaël's 08-13 policy for comparison (2026-10-21)

`policy/policy_acc_08_13_21550.onnx` (g1_velocity `2026-08-13_18-31-06/model_21550`, exported with mjlab's runner
exporter, max |torch − ONNX| 7e-5) is policy index 4: same action class and per-joint scales as the ArmWrench
policies, 495 inputs (no `qacc_ext`). Override files `run/ctl_override_b0813{,_tilt}.yaml`; battery
`tools/queue_b0813.sh` → `run/tests_b0813.txt` (raw logs deleted after conversion, `DROP_BIN=1`: about 8.5 GB per
150 s run).

| mc_mujoco | 08-13 (`model_21550`) | ArmWrenchJointDR (`model_17998`) |
| --- | --- | --- |
| stock model 1 ms / 2.5 ms, 0.5 m/s | no fall, 0.59 / 0.48 m/s | no fall, 0.60 / 0.48 m/s |
| turn schedule, command schedule | no fall; 0.1 m/s gives 0.02 m/s and 0.18 rad/s yaw | no fall; 0.1 m/s gives 0.09 m/s |
| 150 N pushes, 3 seeds | 0 falls in 3056 s | 0 falls in 2970 s |
| 300 N at CoM, ground truth, 6 seeds | 0.44 falls/min | 0.31 |
| 300 N at CoM, Tilt, perfect IMU | 0.98 falls/min | 0.60 |
| Tilt, n4 IMU: turn schedule | no fall, turn in place +0.27 rad/s for +0.5 | no fall, +0.49 |
| Tilt, n4 IMU: command schedule | **falls at 21 s** (0.1 m/s segment) | no fall |
| Tilt, n4 IMU: 300 N at CoM | 1.65 falls/min | 0.65 |

mjlab, same protocol (64 envs × 20 s): 300 N force pushes 0.28 vs 0.38 falls/env-min, turns and controller-model
test 0 falls for both, tracking grids equivalent. With ground-truth state the two policies are equal; with the
estimated state and a noisy IMU, 08-13 degrades much more.

## ArmWrenchRamp without qacc_ext (2026-10-23)

`policy/policy_acc_arm_wrench_ramp_no_qacc_14999.onnx` is policy index 5: ArmWrenchRamp retrained (mjlab task
`...-TrackGated-ArmWrenchRamp-NoQacc`, same seed) with `qacc_ext` removed from the actor, 495 inputs.
`tools/queue_noq.sh` → `run/tests_noq.txt`, same battery as 08-13:

| mc_mujoco | NoQacc 14999 | ArmWrenchRamp 14999 | JointDR 17998 | 08-13 |
| --- | --- | --- | --- | --- |
| stock models, turns, command schedule | no fall | no fall | no fall | no fall |
| 150 N, 3 seeds | **0.62 falls/min** | 0 | 0 | 0 |
| 300 N at CoM, ground truth | **4.14** | 0.81 | 0.31 | 0.44 |
| 300 N, Tilt, perfect IMU | **3.01** | – | 0.60 | 0.98 |
| Tilt n4: turn / command schedule | no fall | no fall | no fall | falls in the schedule |
| 300 N, Tilt n4 | **3.50** | 2.02 | 0.65 | 1.65 |

Same picture as mjlab (300 N force push 3.14 vs 0.61 falls/env-min): without `qacc_ext` the wrench-trained
policy loses its push robustness; command following under IMU noise survives.

## ArmWrenchRamp-Tilt fine-tune (2026-10-26)

`policy/policy_acc_arm_wrench_ramp_tilt_ft_17998.onnx` is policy index 6: ArmWrenchRamp `model_14999`
fine-tuned 3000 iterations in mjlab with the actor's tilt, base velocity and gyro from a torch port of the
Tilt observer run in the loop on a noisy IMU (task `...-ArmWrenchRamp-Tilt`). Same 670 inputs and scales.
`tools/queue_tft.sh` → `run/tests_tft.txt`:

| mc_mujoco | Tilt FT 17998 | JointDR 17998 | ArmWrenchRamp 14999 | 08-13 |
| --- | --- | --- | --- | --- |
| stock models, turns, command schedule | no fall | no fall | no fall | no fall |
| 150 N, 3 seeds | 0 | 0 | 0 | 0 |
| 300 N at CoM, ground truth | 0.59 | 0.31 | 0.81 | 0.44 |
| 300 N, Tilt, perfect IMU | 0.18 | 0.60 | – | 0.98 |
| Tilt n4: turn / command schedule | no fall | no fall | no fall | falls in the schedule |
| 300 N, Tilt n4 | 0.76 | 0.65 | 2.02 | 1.65 |

5–6 falls per cell, so differences under a factor ~2 are counting noise. Forward speed 0.41–0.43 m/s at a
0.5 command (JointDR 0.46). mjlab shows the gain under large base-velocity bias (0.4 m/s + 300 N: 0.80 vs
5.16 falls/min); mc_mujoco's Tilt errors are small (~0.01 m/s), so this battery does not separate them.

## Where Tilt is wrong: velocity bias and slippery feet (2026-10-26)

Tilt FT (index 6) vs JointDR 17998 (index 3), Tilt observer + 4x IMU noise (`nix/tools/queue_tiltwrong.sh`, `queue_slip.sh`).

- `sim_test.obs_vel_bias: [vx, vy, vz]` adds a constant bias to the policy's base_lin_vel observation.
  At 0.2 and 0.4 m/s neither policy falls on straight, turns or the schedule, but both walk at the
  biased speed (0.17 m/s for 0.5 commanded at 0.4). 300 N: 2-3 failures per policy and bias, noise.
- Slippery feet: foot geoms with `priority="1" friction="mu"` (mjlab's G1 has priority 1, friction 0.6;
  without priority MuJoCo takes the floor's 1.0). Failures (QP infeasible, before a fall) per simulated
  minute, 150 N + turns + schedule: mu 0.5 Tilt FT 0.05 / JointDR 0.22; mu 0.4 0.86 / 1.46;
  mu 0.3 both fail within 2 s (noise-free Tilt 29 s, ground truth 68 s).
