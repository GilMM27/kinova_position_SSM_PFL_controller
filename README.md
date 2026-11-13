# Kinova position SSM+PFL controller

`KinovaPositionSSMPFLController` drives the Kinova Gen3 tool to a position target set in `mc_rtc_gui`. It feeds live distance measurements to the `mc_solver::SSMPFLConstraint` in mc_rtc. When measurements are missing or stale, it keeps the last position command and retains the requested target for automatic retry. The controller builds with `mc_rtc` and the header-only [`human_robot_distance_api`](https://github.com/GilMM27/human_robot_distance_api) package. At runtime it needs the Kinova robot module and a publisher of the `HumanRobotDistance::Snapshot` datastore entry using the shared API header.

## Source layout

`src/KinovaPositionSSMPFLController.cpp` implements the controller in a shared library. `src/api.h` exports its class symbols, and `src/lib.cpp` registers the loadable mc_rtc controller. `src/CMakeLists.txt` builds and installs both targets. The SSM+PFL row math and solver registration live in mc_rtc's `mc_tvm` and `mc_solver` libraries. The snapshot type comes from `human_robot_distance_api`; producers and consumers use the same installed header and datastore key.

## Build with the superbuild

The superbuild registers this repository in `extensions/kinova_position_SSM_PFL_controller.cmake` and builds `mc_mujoco` first when SMPL-H logging is enabled, so its public contact snapshot header is available to the controller.

Enable Kinova support and SMPL-H playback in the superbuild, then run `mc_build`. Standalone builds can leave `WITH_SMPLH_CONTACT_LOGGING=OFF` to omit the simulator header dependency. The controller still expects the Kinova robot, its `tool_frame`, and the monitored joint and region names.

With contact logging enabled, `SMPLH_selected_point_torque_force_N` and `SMPLH_selected_point_mujoco_force_N` provide a pointwise proxy comparison for the selected closest region/link. Use `SMPLH_selected_point_force_valid` to identify comparable samples; `SMPLH_selected_point_force_difference_N` is their signed difference. `SMPLH_selected_point_ForceMaxN` follows that region's configured limit. Regional `ForceMaxN` and exceedance use the solved MuJoCo force sum (`SMPLH_<region>_positive_normal_sum_N`). These simulation estimates are not physical force-limit measurements.

The smoke test requires the Kinova robot module and is disabled by default. Configure with `-DBUILD_TESTING=ON` to run it; the shared API package must be installed before configuring.

## Run

Set `MainRobot: Kinova`, `Enabled: KinovaPositionSSMPFLController`, and `Timestep: 0.001` in `mc_rtc.yaml`. Copy `examples/ssm_pfl_example.yaml` to `~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml` and set the 14 `Regions` values under the `constraints:` entry with `type: ssmPfl`. Omit its `Weight` for hard speed rows, or set a positive weight for soft rows. Keep `SSMPFL.MaxSampleAgeSeconds` and `SSMPFL.MaxModelWitnessErrorM` in the controller's input configuration. The controller uses sampled human velocity for both moving and held playback. Held playback supplies a valid zero velocity. Missing or invalid measurements pause command updates; the requested GUI target remains and is retried when valid measurements return. A failed hard QP likewise leaves the last command in place and is retried on the next tick.

`PositionTask.Weight` and `PositionTask.Stiffness` control the tool task. `PostureTask.Weight` and `PostureTask.Stiffness` control the weak posture preference. In `mc_rtc_gui`, move **Tasks → KinovaToolPosition → pos_target** to set the world-frame XYZ target.

The example's force and stiffness values are unverified simulation values, not calibrated safety limits.
