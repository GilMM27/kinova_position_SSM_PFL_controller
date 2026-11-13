#include "../src/KinovaPositionSSMPFLController.h"
#include <mc_rbdyn/RobotLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_tvm/Robot.h>
#include <cmath>
#include <stdexcept>
#include <array>

int main(int argc, char ** argv)
{
  if(argc != 2) throw std::runtime_error("expected SSMPFL configuration path");
  mc_rtc::Configuration config;
  config.load(std::string(argv[1]));
  config.add("PositionTask", mc_rtc::Configuration::fromYAMLData("Weight: 5.0\nStiffness: 2.0"));
  if(std::abs(config("PositionTask")("Weight", 0.0) - 5.0) > 1e-12)
    throw std::runtime_error("example position weight is not 5");
  if(std::abs(config("PositionTask")("Stiffness", 0.0) - 2.0) > 1e-12)
    throw std::runtime_error("example position stiffness is not 2");
  config("PositionTask").remove("Weight"); // Exercise the controller default separately.
  config("PositionTask").remove("Stiffness");
  auto module = mc_rbdyn::RobotLoader::get_robot_module("Kinova");
  KinovaPositionSSMPFLController controller(module, 0.001, config);
  if(controller.solver().backend() != mc_solver::QPSolver::Backend::TVM)
    throw std::runtime_error("position controller did not select TVM");
  if(!controller.robots().hasRobot("ground"))
    throw std::runtime_error("position controller did not load ground");
  const auto & tasks = controller.solver().tasks();
  auto * task = tasks.size() == 2 ? dynamic_cast<mc_tasks::PositionTask *>(tasks.front()) : nullptr;
  auto * posture = tasks.size() == 2 ? dynamic_cast<mc_tasks::PostureTask *>(tasks.back()) : nullptr;
  if(!task || task->name() != "KinovaToolPosition")
    throw std::runtime_error("unexpected position task setup");
  if(!posture || posture->name() != "KinovaPreferredPosture")
    throw std::runtime_error("preferred posture task is missing");
  if(std::abs(task->weight() - 5.0) > 1e-12)
    throw std::runtime_error("position task default weight is not 5");
  if(std::abs(task->stiffness() - 2.0) > 1e-12)
    throw std::runtime_error("position task default stiffness is not 2");
  if(std::abs(posture->weight() - 0.1) > 1e-12 || std::abs(posture->stiffness() - 1.0) > 1e-12)
    throw std::runtime_error("preferred posture task default gains are incorrect");
  auto displacedPosture = controller.robot().q();
  displacedPosture.at(controller.robot().jointIndexByName("joint_1")).at(0) += 0.1;
  posture->posture(displacedPosture);
  controller.reset({controller.robot().q()});
  if(posture->posture() != controller.robot().q())
    throw std::runtime_error("preferred posture was not anchored at reset");
  const Eigen::Vector3d initial = controller.robot().frame("tool_frame").position().translation();
  if((task->position() - initial).norm() > 1e-10)
    throw std::runtime_error("position target was not initialized to tool position");
  const Eigen::Vector3d initialTarget = initial + Eigen::Vector3d(0.1, 0.0, 0.0);
  const auto initialQ = controller.robot().q();
  task->position(initialTarget);
  if(!controller.run()) throw std::runtime_error("controller failed without a distance sample");
  if((task->position() - initialTarget).norm() > 1e-10 || controller.robot().q() != initialQ)
    throw std::runtime_error("missing distance sample changed the target or command");

  human_robot_distance::Snapshot snapshot;
  snapshot.ready = true;
  snapshot.sample_simulation_time = 0.001;
  snapshot.sequence = 1;
  const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                                 "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                                 "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                                 "right_foot"}};
  const Eigen::Vector3d witness = controller.robot().bodyPosW("bracelet_link").translation();
  for(const char * region : regions)
    for(int joint = 1; joint <= 7; ++joint)
    {
      human_robot_distance::RegionDistance sample;
      sample.region = region;
      sample.link = "joint_" + std::to_string(joint);
      sample.distance = 10.0;
      sample.robot_body = "bracelet_link";
      sample.robot_point = {{witness.x(), witness.y(), witness.z()}};
      sample.direction = {{1.0, 0.0, 0.0}};
      sample.closest_points_valid = true;
      sample.direction_valid = true;
      sample.human_velocity = {{0.02, 0.0, 0.0}};
      sample.human_velocity_valid = true;
      snapshot.regions.push_back(sample);
    }
  controller.datastore().make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, snapshot);
  const Eigen::Vector3d target = task->position() + Eigen::Vector3d(0.02, 0.0, 0.0);
  task->position(target);
  if(!controller.run()) throw std::runtime_error("controller failed with valid human velocity");
  if((task->position() - target).norm() > 1e-10)
    throw std::runtime_error("valid sample unexpectedly cancelled GUI target");

  snapshot.sample_simulation_time = -1.0;
  controller.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  const auto beforeStaleQ = controller.robot().q();
  if(!controller.run()) throw std::runtime_error("controller failed while holding on stale sample");
  if((task->position() - target).norm() > 1e-10 || controller.robot().q() != beforeStaleQ)
    throw std::runtime_error("stale sample changed the target or command");

  snapshot.sample_simulation_time = 0.003;
  snapshot.regions.front().human_velocity_valid = false;
  controller.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  task->position(target);
  if(!controller.run()) throw std::runtime_error("controller failed while holding on invalid human velocity");
  if((task->position() - target).norm() > 1e-10 || controller.robot().q() != beforeStaleQ)
    throw std::runtime_error("invalid human velocity changed the target or command");
  snapshot.sample_simulation_time = 0.004;
  snapshot.regions.front().human_velocity_valid = true;
  controller.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  if(!controller.run() || (task->position() - target).norm() > 1e-10)
    throw std::runtime_error("valid measurements did not restore the saved target");

  config("constraints")[0].remove("Weight"); // Exercise hard speed rows without slack.
  config("PositionTask").add("Weight", 7.5);
  config("PositionTask").add("Stiffness", 5.0);
  KinovaPositionSSMPFLController heldController(module, 0.001, config);
  heldController.reset({heldController.robot().q()});
  snapshot.playback_frozen = true;
  for(auto & sample : snapshot.regions)
  {
    sample.human_velocity = {{0.0, 0.0, 0.0}};
    sample.human_velocity_valid = true;
  }
  snapshot.sample_simulation_time = 0.0;
  const Eigen::Vector3d heldWitness = heldController.robot().bodyPosW("bracelet_link").translation();
  for(auto & sample : snapshot.regions)
    sample.robot_point = {{heldWitness.x(), heldWitness.y(), heldWitness.z()}};
  heldController.datastore().make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, snapshot);
  auto * heldTask = dynamic_cast<mc_tasks::PositionTask *>(heldController.solver().tasks().front());
  if(!heldTask || std::abs(heldTask->weight() - 7.5) > 1e-12)
    throw std::runtime_error("configured position task weight was not applied");
  if(std::abs(heldTask->stiffness() - 5.0) > 1e-12)
    throw std::runtime_error("configured position task stiffness was not applied");
  const Eigen::Vector3d heldTarget = heldTask->position() + Eigen::Vector3d(0.02, 0.0, 0.0);
  heldTask->position(heldTarget);
  if(!heldController.run()) throw std::runtime_error("held-playback controller failed");
  if((heldTask->position() - heldTarget).norm() > 1e-10)
    throw std::runtime_error("held playback with valid zero velocity was rejected");

  // Make two priority-0 acceleration bounds contradictory for one tick.
  auto & tvmSolver = mc_solver::TVMQPSolver::from_solver(heldController.solver());
  auto acceleration = heldController.robot().tvmRobot().alphaD();
  auto lower = tvmSolver.problem().add(acceleration >= 1e6, {tvm::requirements::PriorityLevel(0)});
  auto upper = tvmSolver.problem().add(acceleration <= -1e6, {tvm::requirements::PriorityLevel(0)});
  const auto beforeFailureQ = heldController.robot().q();
  snapshot.sample_simulation_time = 0.002;
  heldController.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  if(!heldController.run()) throw std::runtime_error("hard QP failure stopped the controller");
  if(heldController.robot().q() != beforeFailureQ || (heldTask->position() - heldTarget).norm() > 1e-10)
    throw std::runtime_error("hard QP failure changed the command or target");
  tvmSolver.problem().remove(*upper);
  tvmSolver.problem().remove(*lower);
  snapshot.sample_simulation_time = 0.003;
  heldController.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  if(!heldController.run() || (heldTask->position() - heldTarget).norm() > 1e-10)
    throw std::runtime_error("hard QP did not recover after conflict removal");

  KinovaPositionSSMPFLController wrongTypeController(module, 0.001, config);
  wrongTypeController.reset({wrongTypeController.robot().q()});
  wrongTypeController.datastore().make<int>(human_robot_distance::datastoreKey, 1);
  auto * wrongTypeTask = dynamic_cast<mc_tasks::PositionTask *>(wrongTypeController.solver().tasks().front());
  if(!wrongTypeTask) throw std::runtime_error("position task is missing");
  const Eigen::Vector3d wrongTypeTarget = wrongTypeTask->position() + Eigen::Vector3d(0.02, 0.0, 0.0);
  wrongTypeTask->position(wrongTypeTarget);
  if(!wrongTypeController.run()) throw std::runtime_error("controller failed on an incompatible snapshot type");
  if((wrongTypeTask->position() - wrongTypeTarget).norm() > 1e-10)
    throw std::runtime_error("incompatible snapshot type changed the target");
}
