# Kinova position SSM+PFL controller

`KinovaPositionSSMPFLController` drives the Kinova Gen3 tool to a position target set in `mc_rtc_gui`. It adds SSM+PFL speed rows to the TVM QP and holds the target when human distance samples are missing or stale. The controller builds with `mc_rtc` and the header-only [`human_robot_distance_api`](https://github.com/GilMM27/human_robot_distance_api) package. At runtime it needs the Kinova robot module and a publisher of the `HumanRobotDistance::Snapshot` datastore entry using the shared API header.

## Source layout

`src/KinovaPositionSSMPFLController.cpp` implements the controller in a shared library. `src/api.h` exports its class symbols, and `src/lib.cpp` registers the loadable mc_rtc controller. `src/CMakeLists.txt` builds and installs both targets. The snapshot type comes from `human_robot_distance_api`; producers and consumers use the same installed header and datastore key.

## Build with the superbuild

The superbuild registers this repository in `extensions/kinova_position_SSM_PFL_controller.cmake` and builds `mc_mujoco` first when SMPL-H logging is enabled. The controller then finds the public contact snapshot header installed by the simulator fork.

Enable Kinova support and SMPL-H playback in the superbuild, then run `mc_build`. Standalone builds can leave `WITH_SMPLH_CONTACT_LOGGING=OFF` to omit the simulator header dependency. The controller still expects the Kinova robot, its `tool_frame`, and the monitored joint and region names.

With contact logging enabled, `SMPLH_<region>_positive_normal_sum_N` is the solved MuJoCo force and `SMPLH_<region>_torque_estimated_normal_N` is the one-contact torque projection. Plot both in `mc_log_ui` and use `SMPLH_<region>_torque_estimate_valid` to select comparable samples. Invalid estimates and their signed differences are NaN. `SMPLH_torque_estimate_relative_residual` diagnoses poor fits. `ForceMaxN` and exceedance continue to use the solved contact-force sum.

The smoke test requires the Kinova robot module and is disabled by default. Configure with `-DBUILD_TESTING=ON` to run it; the shared API package must be installed before configuring.

## Run

Set `MainRobot: Kinova`, `Enabled: KinovaPositionSSMPFLController`, and `Timestep: 0.001` in `mc_rtc.yaml`. Copy `examples/ssm_pfl_example.yaml` to `~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml` and set the 14 `SSMPFL.Regions` values for your experiment. Omit `SSMPFL.Weight` for hard speed rows, or set a positive weight for soft rows. The controller uses the sampled human velocity for both moving and held playback. Held playback supplies a valid zero velocity; missing or invalid velocity causes the controller to hold its target. A legacy `SSMPFL.HumanMode` setting is ignored; remove it from existing controller configurations for clarity.

`PositionTask.Weight` and `PositionTask.Stiffness` control the tool task. `PostureTask.Weight` and `PostureTask.Stiffness` control the weak posture preference. In `mc_rtc_gui`, move **Tasks → KinovaToolPosition → pos_target** to set the world-frame XYZ target.

The example's force and stiffness values are unverified simulation values, not calibrated safety limits.
