#include "KinovaPositionSSMPFLController.h"
#ifdef WITH_SMPLH_CONTACT_LOGGING
#  include <mc_mujoco/SmplhContactForces.h>
#endif

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/plot.h>
#include <RBDyn/MultiBodyConfig.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <utility>

namespace
{
const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                               "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                               "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                               "right_foot"}};
const std::array<const char *, 7> links = {{"joint_1", "joint_2", "joint_3", "joint_4",
                                            "joint_5", "joint_6", "joint_7"}};

// This controller uses the FSM base for its solver setup but runs no FSM states.
mc_rtc::Configuration managedConfig(const mc_rtc::Configuration & config)
{
  mc_rtc::Configuration managed = config;
  if(managed.has("Managed") && !managed("Managed", false))
    mc_rtc::log::error_and_throw("KinovaPositionSSMPFLController requires Managed: true");
  if(!managed.has("Managed")) managed.add("Managed", true);
  return managed;
}
} // namespace

KinovaPositionSSMPFLController::KinovaPositionSSMPFLController(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, managedConfig(config), Backend::TVM)
{
  // Match the sweep controller's TVM dynamics and collision setup. Replace
  // the FSM idle posture with a weak, independently weighted posture task.
  loadRobot(mc_rbdyn::RobotLoader::get_robot_module("env/ground"), "ground");
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 100.0}, 0.95, true));
  solver().addConstraintSet(dynamicsConstraint);
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 70.0});
  auto idlePosture = getPostureTask(robot().name());
  if(!idlePosture) mc_rtc::log::error_and_throw("Kinova position controller requires an actuated robot");
  solver().removeTask(idlePosture);
  solver().setContacts({});

  // Position task which drives the robot's end effector to meet with the GUI target.
  double positionWeight = 5.0;
  double positionStiffness = 2.0;
  if(config.has("PositionTask"))
  {
    const auto taskConfig = config("PositionTask");
    taskConfig("Weight", positionWeight);
    taskConfig("Stiffness", positionStiffness);
  }
  if(!std::isfinite(positionWeight) || positionWeight <= 0)
    mc_rtc::log::error_and_throw("PositionTask.Weight must be a positive finite value");
  if(!std::isfinite(positionStiffness) || positionStiffness <= 0)
    mc_rtc::log::error_and_throw("PositionTask.Stiffness must be a positive finite value");
  // PositionTask supplies its own draggable GUI target. Its constructor uses
  // critical damping for the configured stiffness.
  positionTask_ = std::make_shared<mc_tasks::PositionTask>("tool_frame", robots(), robot().robotIndex(),
                                                            positionStiffness, positionWeight);
  positionTask_->name("KinovaToolPosition");
  solver().addTask(positionTask_);

  // Posture task which avoids the robot from continiuosly spinning when the goal has been reached.
  double postureWeight = 0.1;
  double postureStiffness = 1.0;
  if(config.has("PostureTask"))
  {
    const auto taskConfig = config("PostureTask");
    taskConfig("Weight", postureWeight);
    taskConfig("Stiffness", postureStiffness);
  }
  if(!std::isfinite(postureWeight) || postureWeight <= 0)
    mc_rtc::log::error_and_throw("PostureTask.Weight must be a positive finite value");
  if(!std::isfinite(postureStiffness) || postureStiffness <= 0)
    mc_rtc::log::error_and_throw("PostureTask.Stiffness must be a positive finite value");
  postureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(),
                                                         postureStiffness, postureWeight);
  postureTask_->name("KinovaPreferredPosture");
  solver().addTask(postureTask_);

  for(const auto & constraint : constraints_)
  {
    auto ssmPfl = std::dynamic_pointer_cast<mc_solver::SSMPFLConstraint>(constraint);
    if(ssmPfl)
    {
      if(safetyConstraint_) mc_rtc::log::error_and_throw("Kinova requires exactly one ssmPfl constraint");
      safetyConstraint_ = std::move(ssmPfl);
    }
  }
  if(!safetyConstraint_ || safetyConstraint_->capacity() != regions.size() * links.size())
    mc_rtc::log::error_and_throw("Kinova requires one ssmPfl constraint with capacity 98");
  for(const auto * name : regions)
    if(!safetyConstraint_->regions().count(name))
      mc_rtc::log::error_and_throw("Missing SSM+PFL region {}", name);
  if(!config.has("SSMPFL")) mc_rtc::log::error_and_throw("Missing SSMPFL input configuration");
  const auto safety = config("SSMPFL");
  safety("MaxSampleAgeSeconds", maxSampleAge_);
  safety("MaxModelWitnessErrorM", maxModelWitnessError_);
  if(!std::isfinite(maxSampleAge_) || maxSampleAge_ <= 0
     || !std::isfinite(maxModelWitnessError_) || maxModelWitnessError_ <= 0)
    mc_rtc::log::error_and_throw("Invalid SSMPFL tolerance");
#ifdef WITH_SMPLH_CONTACT_LOGGING
  for(size_t i = 0; i < smplh_contact::regions.size(); ++i)
  {
    regionLimitN_[i] = safetyConstraint_->regions().at(smplh_contact::regions[i]).forceMaxN;
    const std::string prefix = std::string("SMPLH_") + smplh_contact::regions[i] + "_";
    logger().addLogEntry(prefix + "positive_normal_sum_N", [this, i]() { return regionNormalN_[i]; });
    logger().addLogEntry(prefix + "positive_normal_peak_N", [this, i]() { return regionPeakN_[i]; });
    logger().addLogEntry(prefix + "ForceMaxN", [this, i]() { return regionLimitN_[i]; });
    logger().addLogEntry(prefix + "exceeded", [this, i]() { return regionExceeded_[i]; });
    logger().addLogEntry(prefix + "peak_exceeded", [this, i]() { return regionPeakN_[i] > regionLimitN_[i]; });
  }
  logger().addLogEntry("SMPLH_selected_point_torque_force_N", [this]() { return selectedTorqueForceN_; });
  logger().addLogEntry("SMPLH_selected_point_mujoco_force_N", [this]() { return selectedMujocoForceN_; });
  logger().addLogEntry("SMPLH_selected_point_force_difference_N", [this]() { return selectedForceDifferenceN_; });
  logger().addLogEntry("SMPLH_selected_point_force_valid", [this]() { return selectedPointForceValid_; });
  logger().addLogEntry("SMPLH_selected_point_ForceMaxN", [this]() {
    if(selectedPointForceValid_ && selectedForceRegion_ >= 0
       && static_cast<size_t>(selectedForceRegion_) < regionLimitN_.size())
      return regionLimitN_[static_cast<size_t>(selectedForceRegion_)];
    if(contactSelection_.valid && contactSelection_.region < regionLimitN_.size())
      return regionLimitN_[contactSelection_.region];
    return std::numeric_limits<double>::quiet_NaN();
  });
  logger().addLogEntry("SMPLH_selected_point_contact_distance_m", [this]() { return selectedContactPointDistanceM_; });
  logger().addLogEntry("SMPLH_selected_point_region_index", [this]() {
    return static_cast<double>(selectedForceRegion_);
  });
  logger().addLogEntry("SMPLH_force_sample_available", [this]() { return forceSampleAvailable_; });
  logger().addLogEntry("SMPLH_external_joint_torque_norm_Nm", [this]() { return externalJointTorqueNormNm_; });
  logger().addLogEntry("SMPLH_external_torque_feedback_enabled", [this]() { return externalTorqueFeedback_; });
  logger().addLogEntry("SMPLH_high_pass_compensation_enabled", [this]() { return highPassCompensation_; });
#endif

  gui()->addElement({"SSM+PFL"},
                    mc_rtc::gui::Label("State", [this]() {
                      return fallback_ ? std::string("Paused: ") + fallbackReason_ : std::string("QP running");
                    }),
                    mc_rtc::gui::Label("Active pairs", [this]() { return std::to_string(activePairs_); }),
                    mc_rtc::gui::Label("Limiting pair", [this]() { return limitingPair_; }),
                    mc_rtc::gui::Label("Closest pair in speed plot", [this]() { return speedPlot_.pair; }),
                    mc_rtc::gui::Label("Closest clearance (m)", [this]() {
                      return speedPlot_.valid ? std::to_string(speedPlot_.distance) : std::string("unavailable");
                    }),
                    mc_rtc::gui::Label("Closest pair soft slack (m/s)", [this]() {
                      return speedPlot_.valid ? std::to_string(pairSoftSlack_[speedPlot_.pairIndex])
                                              : std::string("unavailable");
                    }),
                    mc_rtc::gui::Label("Limiting robot speed allowance (m/s)", [this]() {
                      return limitingPair_ == "none" ? std::string("unavailable")
                                                      : std::to_string(limitingSpeed_);
                    }),
                    mc_rtc::gui::Label("Maximum soft slack (m/s)", [this]() {
                      return std::to_string(maxSoftSlack_);
                    }),
                    mc_rtc::gui::Label("Sample age (s)", [this]() { return std::to_string(sampleAge_); }));
  addSpeedPlot();
  logger().addLogEntry("SSMPFL_active_pairs", [this]() { return activePairs_; });
  logger().addLogEntry("SSMPFL_fallback", [this]() { return fallback_; });
  logger().addLogEntry("SSMPFL_solver_success", [this]() { return solverSuccess_; });
  logger().addLogEntry("SSMPFL_max_soft_slack_mps", [this]() { return maxSoftSlack_; });
  logger().addLogEntry("SSMPFL_sample_age_seconds", [this]() { return sampleAge_; });
  logger().addLogEntry("SSMPFL_limiting_pair", [this]() { return limitingPair_; });
  logger().addLogEntry("SSMPFL_fallback_reason", [this]() { return fallbackReason_; });
  logger().addLogEntry("SSMPFL_closest_pair", [this]() { return speedPlot_.pair; });
  logger().addLogEntry("SSMPFL_closest_distance_m", [this]() {
    return speedPlot_.valid ? speedPlot_.distance : std::numeric_limits<double>::quiet_NaN();
  });
  for(size_t i = 0; i < pairSpeedBudget_.size(); ++i)
  {
    const std::string prefix = std::string("SSMPFLPair_") + regions[i / links.size()] + "_"
                             + links[i % links.size()] + "_";
    logger().addLogEntry(prefix + "robot_speed_budget_mps", [this, i]() { return pairSpeedBudget_[i]; });
    logger().addLogEntry(prefix + "soft_slack_mps", [this, i]() { return pairSoftSlack_[i]; });
  }
}

KinovaPositionSSMPFLController::~KinovaPositionSSMPFLController() = default;

void KinovaPositionSSMPFLController::addSpeedPlot()
{
  // A single plot follows the geometrically closest pair as snapshots change.
  const auto plotValue = [](bool valid, double value) {
    return valid ? value : std::numeric_limits<double>::quiet_NaN();
  };
  gui()->addPlot("SSM+PFL closest robot approach speed (m/s)",
                 mc_rtc::gui::plot::X("simulation time (s)", [this]() { return time_; }),
                 mc_rtc::gui::plot::Y("Current model robot approach", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.current);
                 }, mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("QP next robot approach", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.predicted);
                 }, mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Sampled MuJoCo robot approach (delayed)", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.sampled);
                 }, mc_rtc::gui::Color::Magenta, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Speed limit", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.limit);
                 }, mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Solid));
}

void KinovaPositionSSMPFLController::reset(const mc_control::ControllerResetData & data)
{
  mc_control::fsm::Controller::reset(data);
  // Start without a commanded jump, even after a MuJoCo reset.
  positionTask_->reset();
  postureTask_->posture(robot().q());
  safetyConstraint_->clear();
  time_ = 0;
  sampleAge_ = 0;
  fallback_ = false;
  solverSuccess_ = true;
  activePairs_ = 0;
  maxSoftSlack_ = 0;
  limitingSpeed_ = 0;
  limitingPair_ = "none";
  fallbackReason_.clear();
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
#ifdef WITH_SMPLH_CONTACT_LOGGING
  regionNormalN_.fill(0);
  regionPeakN_.fill(0);
  regionExceeded_.fill(false);
  forceSampleAvailable_ = false;
  externalTorqueFeedback_ = false;
  highPassCompensation_ = false;
  externalJointTorqueNormNm_ = 0;
  contactSelection_ = {};
  selectedPointForceValid_ = false;
  selectedForceRegion_ = -1;
  selectedTorqueForceN_ = selectedMujocoForceN_ = selectedForceDifferenceN_ = selectedContactPointDistanceM_
      = std::numeric_limits<double>::quiet_NaN();
  publishContactSelection();
#endif
}

#ifdef WITH_SMPLH_CONTACT_LOGGING
void KinovaPositionSSMPFLController::publishContactSelection()
{
  auto & store = datastore();
  if(!store.has(smplh_contact::selectionDatastoreKey))
    store.make<smplh_contact::Selection>(smplh_contact::selectionDatastoreKey);
  store.assign<smplh_contact::Selection>(smplh_contact::selectionDatastoreKey, contactSelection_);
}

void KinovaPositionSSMPFLController::updateContactForces()
{
  regionNormalN_.fill(0);
  regionPeakN_.fill(0);
  selectedPointForceValid_ = false;
  selectedForceRegion_ = -1;
  selectedTorqueForceN_ = selectedMujocoForceN_ = selectedForceDifferenceN_ = selectedContactPointDistanceM_
      = std::numeric_limits<double>::quiet_NaN();
  forceSampleAvailable_ = false;
  if(datastore().has(smplh_contact::datastoreKey))
  {
    const auto & forces = datastore().get<smplh_contact::Snapshot>(smplh_contact::datastoreKey);
    forceSampleAvailable_ = forces.valid;
    if(forceSampleAvailable_)
    {
      regionNormalN_ = forces.positiveNormalSumN;
      regionPeakN_ = forces.peakPositiveNormalSumN;
      selectedPointForceValid_ = forces.selectedPointValid && forces.selectedRegion < regionLimitN_.size();
      selectedForceRegion_ = selectedPointForceValid_ ? static_cast<int>(forces.selectedRegion) : -1;
      if(selectedPointForceValid_)
      {
        selectedTorqueForceN_ = forces.selectedTorqueForceN;
        selectedMujocoForceN_ = forces.selectedMujocoForceN;
        selectedForceDifferenceN_ = forces.selectedForceDifferenceN;
        selectedContactPointDistanceM_ = forces.selectedContactPointDistanceM;
      }
    }
    externalTorqueFeedback_ = forces.externalTorqueFeedback;
    highPassCompensation_ = forces.highPassCompensation;
    externalJointTorqueNormNm_ = forces.externalJointTorqueNormNm;
  }
  for(size_t i = 0; i < regionNormalN_.size(); ++i)
    regionExceeded_[i] = forceSampleAvailable_ && regionNormalN_[i] > regionLimitN_[i];
}
#endif

const human_robot_distance::Snapshot * KinovaPositionSSMPFLController::distanceSnapshot() const
{
  static const std::string key = human_robot_distance::datastoreKey;
  if(!datastore().has(key)) return nullptr;
  try
  {
    const auto & snapshot = datastore().get<human_robot_distance::Snapshot>(key);
    return snapshot.ready && snapshot.error.empty() ? &snapshot : nullptr;
  }
  catch(const std::exception &)
  {
    return nullptr; // Another publisher uses this key with an incompatible type.
  }
}

void KinovaPositionSSMPFLController::pauseMotion(const std::string & reason)
{
  if(!fallback_ || fallbackReason_ != reason) mc_rtc::log::warning("SSM+PFL pausing command: {}", reason);
  fallback_ = true;
  fallbackReason_ = reason;
  activePairs_ = 0;
  limitingPair_ = "none";
  limitingSpeed_ = 0;
  maxSoftSlack_ = 0;
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
#ifdef WITH_SMPLH_CONTACT_LOGGING
  contactSelection_ = {};
  publishContactSelection();
#endif
  // No QP is run for this tick, so the last robot position command is retained.
  // Keep the GUI target for automatic recovery when measurements return.
  safetyConstraint_->clear();
}

bool KinovaPositionSSMPFLController::updateSafetyRows()
{
#ifdef WITH_SMPLH_CONTACT_LOGGING
  contactSelection_ = {};
#endif
  safetyConstraint_->clear();
  activePairs_ = 0;
  limitingPair_ = "none";
  limitingSpeed_ = 0;
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) { pauseMotion("distance snapshot unavailable or worker error"); return false; }
  sampleAge_ = time_ - snapshot->sample_simulation_time;
  if(!std::isfinite(sampleAge_) || sampleAge_ < -solver().dt() || sampleAge_ > maxSampleAge_)
  { pauseMotion("distance sample stale or timestamp invalid"); return false; }
  if(snapshot->regions.size() != regions.size() * links.size())
  { pauseMotion("missing monitored region/link pairs"); return false; }
  std::map<std::pair<std::string, std::string>, const human_robot_distance::RegionDistance *> samples;
  for(const auto & sample : snapshot->regions)
    if(!samples.emplace(std::make_pair(sample.region, sample.link), &sample).second)
    { pauseMotion("duplicate region/link pair"); return false; }

  std::vector<mc_solver::SSMPFLConstraint::Measurement> measurements;
  measurements.reserve(regions.size() * links.size());
  double closestDistance = std::numeric_limits<double>::infinity();
  size_t closestIndex = 0;
  for(size_t i = 0; i < regions.size() * links.size(); ++i)
  {
    const std::string region(regions[i / links.size()]);
    const std::string label = region + "/" + links[i % links.size()];
    const auto found = samples.find({region, links[i % links.size()]});
    if(found == samples.end()) { pauseMotion("missing pair " + label); return false; }
    const auto & sample = *found->second;
    if(!std::isfinite(sample.distance) || sample.distance < 0
       || !sample.closest_points_valid || !sample.direction_valid || sample.robot_body.empty())
    { pauseMotion("invalid distance or witness at " + label); return false; }
    if(!sample.human_velocity_valid)
    { pauseMotion("human velocity unavailable at " + label); return false; }
    Eigen::Vector3d normal(-sample.direction[0], -sample.direction[1], -sample.direction[2]);
    const double length = normal.norm();
    if(!normal.allFinite() || !std::isfinite(length) || length < 1e-9)
    { pauseMotion("invalid normal at " + label); return false; }
    normal /= length;
    const Eigen::Vector3d local(sample.robot_point_body_local[0], sample.robot_point_body_local[1],
                                sample.robot_point_body_local[2]);
    if(!local.allFinite()) { pauseMotion("invalid local witness at " + label); return false; }
    try { robot().bodyIndexByName(sample.robot_body); }
    catch(const std::exception &) { pauseMotion("unknown robot body at " + label); return false; }
    const auto & bodyPose = robot().bodyPosW(sample.robot_body);
    const Eigen::Vector3d modelWitness = bodyPose.rotation().transpose() * local + bodyPose.translation();
    const Eigen::Vector3d sampledWitness(sample.robot_point[0], sample.robot_point[1], sample.robot_point[2]);
    if(!sampledWitness.allFinite() || !std::isfinite((modelWitness - sampledWitness).norm())
       || (modelWitness - sampledWitness).norm() > maxModelWitnessError_)
    { pauseMotion("robot model/witness disagreement at " + label); return false; }
    const Eigen::Vector3d humanVelocity(sample.human_velocity[0], sample.human_velocity[1],
                                        sample.human_velocity[2]);
    if(!humanVelocity.allFinite()) { pauseMotion("invalid human velocity at " + label); return false; }
    measurements.push_back({region, sample.robot_body, local, normal, humanVelocity, sample.distance});
    if(sample.distance < closestDistance) { closestDistance = sample.distance; closestIndex = i; }
  }
  try { safetyConstraint_->setMeasurements(measurements); }
  catch(const std::exception & e) { pauseMotion(std::string("invalid SSM+PFL row: ") + e.what()); return false; }
  const auto & rowData = safetyConstraint_->rowData();
  double smallestMargin = std::numeric_limits<double>::infinity();
  for(size_t i = 0; i < rowData.size(); ++i)
  {
    const auto & row = rowData[i];
    const std::string label = std::string(regions[i / links.size()]) + "/" + links[i % links.size()];
    pairSpeedBudget_[i] = row.robotSpeedBudget;
    const double margin = row.robotSpeedBudget - row.robotApproach;
    if(margin < smallestMargin) { smallestMargin = margin; limitingPair_ = label; limitingSpeed_ = row.robotSpeedBudget; }
  }
  activePairs_ = static_cast<int>(rowData.size());
  const auto & closest = *samples.at({regions[closestIndex / links.size()], links[closestIndex % links.size()]});
  const auto & closestRow = rowData[closestIndex];
  speedPlot_.pairIndex = closestIndex;
  speedPlot_.pair = std::string(regions[closestIndex / links.size()]) + "/" + links[closestIndex % links.size()];
  speedPlot_.distance = closest.distance;
  speedPlot_.current = closestRow.robotApproach;
  speedPlot_.limit = closestRow.robotSpeedBudget;
  speedPlot_.drift = closestRow.jacobianDrift;
  speedPlot_.directional = closestRow.directionalJacobian;
  speedPlot_.sampled = std::numeric_limits<double>::quiet_NaN();
  if(closest.robot_velocity_valid)
  {
    const Eigen::Vector3d sampledVelocity(closest.robot_velocity[0], closest.robot_velocity[1],
                                          closest.robot_velocity[2]);
    if(sampledVelocity.allFinite()) speedPlot_.sampled = measurements[closestIndex].normal.dot(sampledVelocity);
  }
#ifdef WITH_SMPLH_CONTACT_LOGGING
  const std::string closestRegion(regions[closestIndex / links.size()]);
  const auto regionIt = std::find(smplh_contact::regions.begin(), smplh_contact::regions.end(), closestRegion);
  if(regionIt != smplh_contact::regions.end())
  {
    contactSelection_.valid = true;
    contactSelection_.time = time_;
    contactSelection_.region = static_cast<size_t>(std::distance(smplh_contact::regions.begin(), regionIt));
    contactSelection_.robotBody = closest.robot_body;
    for(int k = 0; k < 3; ++k)
    {
      contactSelection_.position[k] = closest.robot_point[k];
      contactSelection_.normal[k] = measurements[closestIndex].normal[k];
    }
  }
  publishContactSelection();
#endif
  fallback_ = false;
  fallbackReason_.clear();
  speedPlot_.valid = true;
  return true;
}

bool KinovaPositionSSMPFLController::run()
{
#ifdef WITH_SMPLH_CONTACT_LOGGING
  updateContactForces();
#endif
  // Keep running the FSM lifecycle and accepting GUI targets during a pause.
  // The robot command changes only after a QP with current safety rows succeeds.
  mc_control::fsm::Controller::run(mc_solver::FeedbackType::SkipQP);
  time_ += solver().dt();
  const bool measurementsValid = updateSafetyRows();
  updateContacts();
  if(!measurementsValid) { solverSuccess_ = false; return true; }
  solverSuccess_ = solver().run(mc_solver::FeedbackType::None);
  if(!solverSuccess_)
  {
    if(!fallback_ || fallbackReason_ != "QP solve failed") mc_rtc::log::warning("SSM+PFL QP solve failed; retrying next tick");
    fallback_ = true;
    fallbackReason_ = "QP solve failed";
    speedPlot_.predicted = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  maxSoftSlack_ = 0;
  if(!fallback_)
  {
    for(int i = 0; i < static_cast<int>(regions.size() * links.size()); ++i)
    {
      pairSoftSlack_[i] = safetyConstraint_->softSlack(i);
      maxSoftSlack_ = std::max(maxSoftSlack_, pairSoftSlack_[i]);
    }
    // Predict the plotted point's next approach speed from the QP acceleration.
    const Eigen::VectorXd acceleration = rbd::dofToVector(robot().mb(), robot().alphaD());
    speedPlot_.predicted = speedPlot_.current + solver().dt()
        * (speedPlot_.directional.dot(acceleration) + speedPlot_.drift);
  }
  return true;
}

