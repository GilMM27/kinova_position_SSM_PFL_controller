# Kinova position SSM+PFL controller

`KinovaPositionSSMPFLController` drives the Kinova Gen3 tool to a position target set in `mc_rtc_gui`. It adds SSM+PFL speed rows to the TVM QP and holds the target when human distance samples are missing or stale. The controller needs `mc_rtc`, the Kinova robot module, and the SMPL-H distance snapshot header installed by the superbuild's `mc_mujoco` playback bridge.

## Source layout

`src/KinovaPositionSSMPFLController.cpp` implements the controller in a shared library. `src/api.h` exports its class symbols, and `src/lib.cpp` registers the loadable mc_rtc controller. `src/CMakeLists.txt` builds and installs both targets.

## Build with the superbuild

The superbuild registers this repository in `extensions/zz_kinova_position_ssm_pfl_controller.cmake`. It clones `GilMM27/kinova_position_SSM_PFL_controller` by default. To build an existing checkout, configure the superbuild with:

```sh
cmake -S safe-rl-qp-mc-rtc-superbuild -B /path/to/superbuild-build \
  -DWITH_SMPLH_PLAYBACK=ON \
  -DKINOVA_POSITION_SSM_PFL_SOURCE_DIR=/home/gilberto/repos/kinova_position_SSM_PFL_controller
cmake --build /path/to/superbuild-build --target kinova_position_SSM_PFL_controller
```

## Run

Set `MainRobot: Kinova`, `Enabled: KinovaPositionSSMPFLController`, and `Timestep: 0.001` in `mc_rtc.yaml`. Copy `examples/ssm_pfl_example.yaml` to `~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml` and set the 14 `SSMPFL.Regions` values for your experiment. Omit `SSMPFL.Weight` for hard speed rows, or set a positive weight for soft rows. The controller uses the sampled human velocity for both moving and held playback. Held playback supplies a valid zero velocity; missing or invalid velocity causes the controller to hold its target. A legacy `SSMPFL.HumanMode` setting is ignored; remove it from existing controller configurations for clarity.

`PositionTask.Weight` and `PositionTask.Stiffness` control the tool task. `PostureTask.Weight` and `PostureTask.Stiffness` control the weak posture preference. In `mc_rtc_gui`, move **Tasks → KinovaToolPosition → pos_target** to set the world-frame XYZ target.

The example's force and stiffness values are unverified simulation values, not calibrated safety limits.
