#pragma once

#include "api.h"

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <human_robot_distance_api/DistanceSnapshot.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_solver/SSMPFLConstraint.h>
#ifdef WITH_SMPLH_CONTACT_LOGGING
#  include <mc_mujoco/SmplhContactForces.h>
#endif

#include <array>
#include <memory>
#include <string>

/** GUI-controlled tool XYZ task with pointwise, soft SSM+PFL speed rows. */
struct KinovaPositionSSMPFLController_DLLAPI KinovaPositionSSMPFLController : public mc_control::fsm::Controller
{
  KinovaPositionSSMPFLController(mc_rbdyn::RobotModulePtr rm, double dt,
                                 const mc_rtc::Configuration & config);
  ~KinovaPositionSSMPFLController() override;

  void reset(const mc_control::ControllerResetData & data) override;
  bool run() override;

private:
  const human_robot_distance::Snapshot * distanceSnapshot() const;
  bool updateSafetyRows();
  void pauseMotion(const std::string & reason);
  void addSpeedPlot();

  std::shared_ptr<mc_tasks::PositionTask> positionTask_;
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  std::shared_ptr<mc_solver::SSMPFLConstraint> safetyConstraint_;
#ifdef WITH_SMPLH_CONTACT_LOGGING
  std::array<double, 14> regionNormalN_{};
  std::array<double, 14> regionPeakN_{};
  std::array<double, 14> regionLimitN_{};
  std::array<bool, 14> regionExceeded_{};
  bool forceSampleAvailable_ = false;
  bool externalTorqueFeedback_ = false;
  bool highPassCompensation_ = false;
  double externalJointTorqueNormNm_ = 0;
  smplh_contact::Selection contactSelection_;
  double selectedTorqueForceN_ = 0;
  double selectedMujocoForceN_ = 0;
  double selectedForceDifferenceN_ = 0;
  double selectedContactPointDistanceM_ = 0;
  int selectedForceRegion_ = -1;
  bool selectedPointForceValid_ = false;
  void updateContactForces();
  void publishContactSelection();
#endif
  bool fallback_ = false;
  bool solverSuccess_ = true;
  double time_ = 0;
  double sampleAge_ = 0;
  double maxSampleAge_ = 0.1;
  double maxModelWitnessError_ = 0.1;
  int activePairs_ = 0;
  double maxSoftSlack_ = 0;
  double limitingSpeed_ = 0;
  std::array<double, 98> pairSpeedBudget_{};
  std::array<double, 98> pairSoftSlack_{};
  // The limiting pair has the smallest speed margin. The plotted pair has
  // the smallest geometric clearance; they need not be the same pair.
  std::string limitingPair_ = "none";
  std::string fallbackReason_;
  struct SpeedPlotSample
  {
    // Invalid samples plot as gaps instead of retaining an old pair's values.
    bool valid = false;
    size_t pairIndex = 0;
    std::string pair = "none";
    double distance = 0;
    double current = 0;
    double predicted = 0;
    double sampled = 0;
    double limit = 0;
    double humanApproach = 0;
    double drift = 0;
    Eigen::RowVectorXd directional;
  } speedPlot_;
};
