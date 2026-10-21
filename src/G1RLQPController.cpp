#include "G1RLQPController.h"

#include <RBDyn/FD.h>
#include <RBDyn/MultiBodyConfig.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <mc_joystick_plugin/joystick_inputs.h>
#include <mc_tvm/Robot.h>

G1RLQPController::G1RLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  positionPolicyIndex_ = size_t(config_("position_policy_index", 0));
  accelerationPolicyIndex_ = size_t(config_("acceleration_policy_index", 1));
  currentPolicyIndex = positionPolicyIndex_;

  // Initialize Constraints
  // Acc-CBF joint-limit gains (defaults in the header), overridable for sim-to-sim tests.
  if(config_.has("joint_limit_cbf"))
  {
    const auto cbf = config_("joint_limit_cbf");
    cbf("zeta", zeta_jointLimit_);
    cbf("lambda", lambda_jointLimit_);
    cbf("ds_percent", dsPercent_);
    mc_rtc::log::info("[G1RLQPController] joint-limit CBF: zeta={}, lambda={}, ds={}", zeta_jointLimit_,
                      lambda_jointLimit_, dsPercent_);
  }
  // selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(
      robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true, true));
  solver().addConstraintSet(dynamicsConstraint);

  initializeRobot();
  initializeRLPolicy();
  loadSimTest();
  datastore().make_call("G1RLQPController::anchorFrame",
                        [this](const mc_rbdyn::Robot & r) { return contactAnchor(r); });

  postureTaskRL_ = getPostureTask(robot().name());
  if(!postureTaskRL_)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] PostureTask not found for robot {}", robot().name());
  }

  // Each FSM state owns exactly one RL task. The initial state will add the
  // TorqueJointTask; the acceleration state will add the PostureTask.
  solver().removeTask(postureTaskRL_);
  initializePositionControl();

  addGui();
  addLog();

  mc_rtc::log::success(
      "[G1RLQPController] Position/acceleration policy controller initialized");
}

bool G1RLQPController::run()
{
  if(printLimits_)
  {
    computeLimits();
  }

  updateSimTest();

  const bool qpSuccess = mc_control::fsm::Controller::run(
      mc_solver::FeedbackType::ClosedLoopIntegrateReal);

  if(rlmode == POSITION && byPassQPControl())
  {
    return true;
  }

  updateAccelerationDiagnostics();
  // The QP only feeds the diagnostics when its torques are replaced; a failed solve must not stop the run.
  const bool ok = qpSuccess || (mjlabActionLaw_ && rlmode == ACCELERATION);

  if(datastore().has("Joystick::connected") && datastore().get<bool>("Joystick::connected"))
  {
    // mc_rtc::log::info("[HRP5pRLQPController] Joystick connected, using joystick inputs for velocity command");
    RLuseJoyStickInputs();
    if(!useJoystick_) useJoystick_ = true;
  }
  else
  {
    // mc_rtc::log::info("[HRP5pRLQPController] Joystick not connected, using default velocity command");
    if(useJoystick_) currentVelCmd = Eigen::Vector3d::Zero();
    useJoystick_ = false;
  }

  return ok;
}

void G1RLQPController::activateQPControl(bool activate)
{
  if(rlmode == ACCELERATION && !activate)
  {
    mc_rtc::log::warning(
        "[G1RLQPController] The QP cannot be bypassed in acceleration mode");
    return;
  }
  useQP_ = activate;
}

void G1RLQPController::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  currentAction.setZero();
  currentActionScaled.setZero();
  q_rl = q_zero;
  qddRL_.setZero();
  qddRefFull_.setZero();

  if(rlmode == POSITION && torqueJointTaskRL_)
  {
    torqueJointTaskRL_->setPosTarget(q_rl);
    torqueJointTaskRL_->setVelTarget(Eigen::VectorXd::Zero(nbActuatedJoints));
  }
  else if(postureTaskRL_)
  {
    postureTaskRL_->setGains(0.0, 0.0);
    postureTaskRL_->refVel(Eigen::VectorXd::Zero(nbActuatedJoints));
    postureTaskRL_->refAccel(qddRL_);
  }
}

void G1RLQPController::initializeRobot()
{
  useQP_ = true;
  // mc_mujoco sends robot().mbc().jointTorque to the motors only in this mode.
  datastore().make<std::string>("ControlMode", "Torque");

  // get the joints order (urdf) depending on the robot used
  robotName_ = robot().name();
  jointNames = robot().refJointOrder(); // Get the joint names in the order used by the robot state
  nbActuatedJoints = jointNames.size();

  q_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  q_zero = Eigen::VectorXd::Zero(nbActuatedJoints);
  actionScale = Eigen::VectorXd::Zero(nbActuatedJoints);
  currentActionScaled = Eigen::VectorXd::Zero(nbActuatedJoints);
  kp_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kd_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kpBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kdBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
}

void G1RLQPController::loadPolicyConfiguration(size_t policyIndex)
{
  const auto common = config_("policy_common")[0];
  const auto policy = config_("policies")[policyIndex];

  useQP_ = policy("use_QP", true);
  observeQaccExt_ = policy("qacc_ext_observation", false);
  pdGainsRatio_ = common("pd_gains_ratio", 1.0);

  const std::map<std::string, double> actionScale_map =
      policy.has("action_scale")
          ? policy("action_scale")
          : common("action_scale");
  const std::map<std::string, double> kp_map = common("kp");
  const std::map<std::string, double> kd_map = common("kd");
  q0_map_ = common("q0");
  const std::vector<double> velocityCommand =
      policy("velocity_command", common("velocity_command", std::vector<double>{0.0, 0.0, 0.0}));

  if(velocityCommand.size() != 3)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] velocity_command must contain exactly 3 values");
  }

  currentVelCmd << velocityCommand[0],
                  velocityCommand[1],
                  velocityCommand[2];

  actionScale.setZero();
  kpBase_.setZero();
  kdBase_.setZero();
  q_zero.setZero();

  // Fill the gain vectors based on the joint names and the configuration maps. If a joint is not found in the map, the
  // joint is skipped. Used for the action scale, useful if the policy doesn't control all the joints.
  auto updateIfExists = [&](auto & target, const auto & map, const std::string & joint_name)
  {
    if(auto it = map.find(joint_name); it != map.end())
    {
      target = it->second;
    }
  };

  for(int i = 0; i < nbActuatedJoints; ++i)
  {
    const auto & joint_name = jointNames[static_cast<size_t>(i)];
    kpBase_[i] = kp_map.at(joint_name);
    kdBase_[i] = kd_map.at(joint_name);
    q_zero[i] = q0_map_.at(joint_name);
    updateIfExists(actionScale[i], actionScale_map, joint_name);
  }

  q_rl = q_zero;
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
}

void G1RLQPController::initializeRLPolicy()
{
  policyPaths_ = config_("policy_path", std::vector<std::string>{});
  loadedPolicies_.resize(policyPaths_.size());
  switchPolicy(positionPolicyIndex_);
}

void G1RLQPController::switchPolicy(size_t policyIndex)
{
  if(policyIndex >= policyPaths_.size())
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] Policy index {} is missing from policy_path", policyIndex);
  }

  const auto policies = config_("policies");
  if(policyIndex >= policies.size())
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] Policy index {} is missing from policies", policyIndex);
  }

  if(policyPaths_[policyIndex].empty())
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] policy_path[{}] is empty; set the ONNX path before launching the controller",
        policyIndex);
  }

  currentPolicyIndex = policyIndex;
  loadPolicyConfiguration(policyIndex);
  configRL();

  currentObservation =
      Eigen::VectorXd::Zero(rlPolicy->getObservationSize());

  currentAction =
      Eigen::VectorXd::Zero(rlPolicy->getActionSize());

  const Eigen::Index jointObsSize =
      static_cast<Eigen::Index>(refJointOrderRLAction.size());

  for(int i = 0; i < HISTORY_SIZE; ++i)
  {
    linVel[i].setZero();
    angVel[i].setZero();
    projectedGravity[i].setZero();
    velCmd[i].setZero();

    jointPos[i] =
        Eigen::VectorXd::Zero(jointObsSize);

    jointVel[i] =
        Eigen::VectorXd::Zero(jointObsSize);

    jointAction[i] =
        Eigen::VectorXd::Zero(currentAction.size());

    qaccExt[i] =
        Eigen::VectorXd::Zero(6 + jointObsSize);
  }

  currentActionScaled.setZero();
  q_rl = q_zero;
  resetRLHistory();
}

void G1RLQPController::resetRLHistory()
{
  initializeRLObservation();

  for(int i = 1; i < HISTORY_SIZE; ++i)
  {
    linVel[i] = linVel[0];
    angVel[i] = angVel[0];
    projectedGravity[i] = projectedGravity[0];
    jointPos[i] = jointPos[0];
    jointVel[i] = jointVel[0];
    jointAction[i] = jointAction[0];
    velCmd[i] = velCmd[0];
    qaccExt[i] = qaccExt[0];
  }
}

void G1RLQPController::initializePositionControl()
{
  const auto policy = config_("policies")[positionPolicyIndex_];
  const double taskWeight = policy("position_task_weight", 1.0);

  if(!std::isfinite(taskWeight) || taskWeight <= 0.0)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] position_task_weight must be finite and positive, got {}", taskWeight);
  }

  if(!torqueJointTaskRL_)
  {
    torqueJointTaskRL_ = std::make_shared<mc_tasks::TorqueJointTask>(
        solver(), robot().robotIndex(), 1.0, taskWeight);
  }
  else
  {
    torqueJointTaskRL_->weight(taskWeight);
  }

  torqueJointTaskRL_->setStiffness(kp_);
  torqueJointTaskRL_->setDamping(kd_);
  torqueJointTaskRL_->enableIntegralTerm(false);
  torqueJointTaskRL_->setDeriveVelocityTargetFromPosition(false);
  torqueJointTaskRL_->setPosTarget(q_rl);
  torqueJointTaskRL_->setVelTarget(Eigen::VectorXd::Zero(nbActuatedJoints));
}

void G1RLQPController::initializeAccelerationControl()
{
  if(!postureTaskRL_)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] Cannot initialize acceleration control: "
        "PostureTask is null");
  }

  const Eigen::Index nrDof =
      static_cast<Eigen::Index>(
          robot().mb().nrDof());

  qddRL_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  qddRefFull_ =
      Eigen::VectorXd::Zero(nrDof);

  const auto policy = config_("policies")[accelerationPolicyIndex_];
  accelerationActionRatio_ = policy("acceleration_action_ratio", 1.0);
  accelerationLimit_ = policy("acceleration_limit", 900.0);

  const double postureWeight = policy("posture_weight", 10.0);

  if(!useQP_)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] Acceleration control requires use_QP: true");
  }
  if(!std::isfinite(accelerationActionRatio_) || accelerationActionRatio_ < 0.0
     || accelerationActionRatio_ > 1.0)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] acceleration_action_ratio must be in [0, 1], got {}",
        accelerationActionRatio_);
  }
  if(!std::isfinite(accelerationLimit_) || accelerationLimit_ <= 0.0)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] acceleration_limit must be finite and positive, got {}",
        accelerationLimit_);
  }
  if(!std::isfinite(postureWeight) || postureWeight <= 0.0)
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] posture_weight must be finite and positive, got {}", postureWeight);
  }

  // Aucune correction de position ni de vitesse.
  postureTaskRL_->setGains(0.0, 0.0);

  // Le poids doit rester non nul pour que refAccel influence le QP.
  postureTaskRL_->weight(postureWeight);

  // TVM's PostureFunction acts on qJoints (29 actuated DoFs), not on the
  // six floating-base DoFs. The mc_rtc public wrapper still has a legacy
  // nrDof assertion, but the vector consumed by TVM must have this size.
  postureTaskRL_->reset();
  postureTaskRL_->refVel(Eigen::VectorXd::Zero(nbActuatedJoints));
  postureTaskRL_->refAccel(qddRL_);

  mc_rtc::log::info(
      "[G1RLQPController] PostureTask configured: "
      "Kp={}, Kd={}, weight={}, actuated DoFs={}, action ratio={}, limit={}",
      postureTaskRL_->stiffness(),
      postureTaskRL_->damping(),
      postureTaskRL_->weight(),
      nbActuatedJoints,
      accelerationActionRatio_,
      accelerationLimit_);
}

void G1RLQPController::enterPositionControl()
{
  switchPolicy(positionPolicyIndex_);
  rlmode = POSITION;

  if(accelerationTaskActive_)
  {
    solver().removeTask(postureTaskRL_);
    accelerationTaskActive_ = false;
  }

  initializePositionControl();
  solver().addTask(torqueJointTaskRL_);
  positionTaskActive_ = true;
}

void G1RLQPController::leavePositionControl()
{
  if(positionTaskActive_)
  {
    solver().removeTask(torqueJointTaskRL_);
    positionTaskActive_ = false;
  }
}

void G1RLQPController::enterAccelerationControl()
{
  switchPolicy(accelerationPolicyIndex_);
  rlmode = ACCELERATION;

  if(positionTaskActive_)
  {
    solver().removeTask(torqueJointTaskRL_);
    positionTaskActive_ = false;
  }

  initializeAccelerationControl();
  solver().addTask(postureTaskRL_);
  accelerationTaskActive_ = true;
}

void G1RLQPController::leaveAccelerationControl()
{
  if(postureTaskRL_)
  {
    qddRL_.setZero();
    qddRefFull_.setZero();
    postureTaskRL_->refAccel(qddRL_);
  }
  if(accelerationTaskActive_)
  {
    solver().removeTask(postureTaskRL_);
    accelerationTaskActive_ = false;
  }
}
// void G1RLQPController::initializeRLPolicy()
// {
//   // load policy specific configuration
//   policyPaths_ = config_("policy_path", std::vector<std::string>{"walking_better_h1.onnx"});
//   configRL();

//   currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
//   currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());

//   initializeRLObservation();

//   // Initialize all history slots (example for this set of observations)
//   // for (int i = 0; i < HISTORY_SIZE; ++i) {
//   //     linVel[i] = linVel[0];
//   //     angVel[i] = angVel[0];
//   //     projectedGravity[i] = projectedGravity[0];
//   //     velCmd[i] = velCmd[0];
//   //     jointPos[i] = jointPos[0];
//   //     jointVel[i] = jointVel[0];
//   //     jointAction[i] = jointAction[0];
//   // }
// }

// void G1RLQPController::initializeRLObservation()
// {
//   // Observation
//   auto & robot = realRobot(robots()[0].name());

//   // Bellow an example of how to fill the observation vector with the robot state, this should be adapted to the
//   // specific policy observation used.

//   // // ---------------- Joint positions and velocities ---------------------------------------
//   // const auto & q_mbc = robot.mbc().q; // MBC order
//   // const auto & q_dot_mbc = robot.mbc().alpha; // MBC order
//   // Eigen::VectorXd q_rlFrameworkOrdered, q_0_rlFrameworkOrdered, q_dot_rlFrameworkOrdered;

//   // if (currentPolicyIndex < 2) // Use all joints as observation
//   // {
//   //   q_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);
//   //   q_0_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);
//   //   q_dot_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);

//   //   for(size_t i = 0; i < jointNames.size(); ++i)
//   //   {
//   //     const auto & joint_name = jointNames[i];

//   //     // Fill mc_rtc ordered vectors
//   //     const double q = q_mbc[robot.jointIndexByName(joint_name)][0];
//   //     const double q_dot = q_dot_mbc[robot.jointIndexByName(joint_name)][0];

//   //     // RL remapping
//   //     int rl_index = mcRtcToRLFrameworkJointMap[i];
//   //     q_rlFrameworkOrdered(rl_index) = q;
//   //     q_0_rlFrameworkOrdered(rl_index) = q_zero(i);
//   //     q_dot_rlFrameworkOrdered(rl_index) = q_dot;
//   //   }
//   // }
//   // else // Use only the joints that are in the action space as observation
//   // {
//   //   const int policyObsJointSize = rlPolicy->getActionSize();
//   //   q_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
//   //   q_0_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
//   //   q_dot_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
//   //   int i = 0;
//   //   for (const auto &joint_name : refJointOrderRLAction)
//   //   {
//   //     q_rlFrameworkOrdered[i] = q_mbc[robot.jointIndexByName(joint_name)][0];
//   //     q_dot_rlFrameworkOrdered[i] = q_dot_mbc[robot.jointIndexByName(joint_name)][0];
//   //     q_0_rlFrameworkOrdered[i] = q0_map_.at(joint_name);
//   //     i++;
//   //   }
//   // }

//   // // gravity, fb linear and angular velocity in floating base frame --------------------------------
//   // const auto & X_0_body = robot.mbc().bodyPosW[robot.mb().bodyIndexByName("Body")];
//   // const auto & bodyVel = robot.mbc().bodyVelB[robot.mb().bodyIndexByName("Body")];
//   // Eigen::Matrix3d R_world_to_body = X_0_body.rotation();
//   // Eigen::Vector3d gravity_b = R_world_to_body * Eigen::Vector3d(0.0, 0.0, -1.0);
//   // Eigen::Vector3d angVel_b = bodyVel.angular();
//   // Eigen::Vector3d linVel_b = bodyVel.linear();

//   // projectedGravity[0] = gravity_b;
//   // angVel[0] = angVel_b;
//   // linVel[0] = linVel_b;
//   // velCmd[0] = currentVelCmd;
//   // jointPos[0] = q_rlFrameworkOrdered - q_0_rlFrameworkOrdered; // Start with current joint positions
//   // jointVel[0] = q_dot_rlFrameworkOrdered;
//   // jointAction[0] = currentAction;
// }

void G1RLQPController::initializeRLObservation()
{
  auto & real_robot = realRobot(robot().name());

  // Met à jour les grandeurs cinématiques associées à l'état observé.
  real_robot.forwardKinematics();
  real_robot.forwardVelocity();

  const auto & q_mbc = real_robot.mbc().q;
  const auto & q_dot_mbc = real_robot.mbc().alpha;

  const Eigen::Index jointObsSize =
      static_cast<Eigen::Index>(refJointOrderRLAction.size());

  if(jointPos[0].size() != jointObsSize)
  {
    jointPos[0] = Eigen::VectorXd::Zero(jointObsSize);
    jointVel[0] = Eigen::VectorXd::Zero(jointObsSize);
  }

  // -------------------------------------------------------------------------
  // Positions et vitesses articulaires dans l'ordre exact de la policy.
  // joint_pos_rel = q - q0
  // joint_vel_rel = qdot - qdot0, avec qdot0 = 0
  // -------------------------------------------------------------------------
  for(size_t rlIndex = 0;
      rlIndex < refJointOrderRLAction.size();
      ++rlIndex)
  {
    const std::string & jointName =
        refJointOrderRLAction[rlIndex];

    const auto jointIndex =
        real_robot.jointIndexByName(jointName);

    const double q =
        q_mbc[jointIndex][0];

    const double qDot =
        q_dot_mbc[jointIndex][0];

    jointPos[0](static_cast<Eigen::Index>(rlIndex)) =
        q - q0_map_.at(jointName);

    jointVel[0](static_cast<Eigen::Index>(rlIndex)) =
        qDot;
  }

  // -------------------------------------------------------------------------
  // État de la base flottante.
  // bodyVelB est exprimée dans le repère de la base.
  // -------------------------------------------------------------------------
  constexpr int rootBodyIndex = 0;

  const auto & X_0_root =
      real_robot.mbc().bodyPosW[rootBodyIndex];

  const auto & rootVelocityB =
      real_robot.mbc().bodyVelB[rootBodyIndex];

  static int baseVelocityDebugCounter = 0;

if(++baseVelocityDebugCounter >= 400)
{


  mc_rtc::log::info(
      "[BASE DEBUG] body={} index={}",
      real_robot.mb().body(rootBodyIndex).name(),
      rootBodyIndex);

  mc_rtc::log::info(
      "[BASE DEBUG] bodyVelB angular=[{:.12e}, {:.12e}, {:.12e}] "
      "linear=[{:.12e}, {:.12e}, {:.12e}]",
      rootVelocityB.angular().x(),
      rootVelocityB.angular().y(),
      rootVelocityB.angular().z(),
      rootVelocityB.linear().x(),
      rootVelocityB.linear().y(),
      rootVelocityB.linear().z());


  baseVelocityDebugCounter = 0;
}

  linVel[0] = rootVelocityB.linear();
  angVel[0] = rootVelocityB.angular();

  // Gravité mondiale exprimée dans le repère de la base.
  projectedGravity[0] =
      X_0_root.rotation()
      * Eigen::Vector3d(0.0, 0.0, -1.0);

  // L'observation "last_action" est l'action précédente non mise à l'échelle.
  jointAction[0] = currentAction;



  // Commande [vx, vy, yaw_rate].
  velCmd[0] = currentVelCmd;

  if(observeQaccExt_) { qaccExt[0] = computeQaccExt(); }
}

void G1RLQPController::loadSimTest()
{
  if(!config_.has("sim_test")) { return; }
  const auto st = config_("sim_test");
  simTestEnabled_ = true;
  st("qacc_ext_gain", qaccExtGain_);
  st("tau_ext_base_force_world", tauExtBaseForceWorld_);
  st("mjlab_action_law", mjlabActionLaw_);
  st("mjlab_tau_ext_cutoff_hz", mjlabTauExtCutoffHz_);
  if(mjlabActionLaw_) { mc_rtc::log::warning("[G1RLQPController] sim_test: MJLab action law replaces the QP torques (tau_ext cutoff {} Hz)", mjlabTauExtCutoffHz_); }
  if(qaccExtGain_ != 1.0) { mc_rtc::log::warning("[G1RLQPController] sim_test: qacc_ext observation scaled by {}", qaccExtGain_); }
  if(st.has("command_schedule"))
  {
    for(const std::vector<double> & row : st("command_schedule"))
    {
      if(row.size() != 4) { mc_rtc::log::error_and_throw("[G1RLQPController] command_schedule rows are [t, vx, vy, wz]"); }
      commandSchedule_.push_back({row[0], row[1], row[2], row[3]});
    }
  }
  if(st.has("pushes"))
  {
    const auto p = st("pushes");
    pushesEnabled_ = true;
    p("body", pushBody_);
    p("at_com", pushAtCom_);
    p("force", pushForce_);
    p("duration", pushDuration_);
    p("interval", pushInterval_);
    pushRng_.seed(static_cast<unsigned>(p("seed", 0)));
    nextPushTime_ = p("start", 2.0);
  }
  mc_rtc::log::info("[G1RLQPController] sim_test: {} command entries, pushes {} (body {}, {}-{} N)",
                    commandSchedule_.size(), pushesEnabled_, pushBody_, pushForce_[0], pushForce_[1]);
}

void G1RLQPController::updateSimTest()
{
  if(!simTestEnabled_ || rlmode != ACCELERATION) { return; }
  const double t = simTestTime_;
  simTestTime_ += timeStep;

  for(const auto & row : commandSchedule_)
  {
    if(row[0] <= t) { currentVelCmd << row[1], row[2], row[3]; }
  }

  if(!pushesEnabled_) { return; }
  auto uniform = [this](const std::array<double, 2> & r)
  { return std::uniform_real_distribution<double>(r[0], r[1])(pushRng_); };
  if(t >= nextPushTime_)
  {
    const double angle = std::uniform_real_distribution<double>(-M_PI, M_PI)(pushRng_);
    const double force = uniform(pushForce_);
    pushForceVec_ << force * std::cos(angle), force * std::sin(angle), 0.0;
    pushEndTime_ = t + uniform(pushDuration_);
    nextPushTime_ = pushEndTime_ + uniform(pushInterval_);
  }
  if(t < pushEndTime_)
  {
    const auto call = robot().name() + "::ApplyForcesOnBody";
    Eigen::Vector3d pushPoint = Eigen::Vector3d::Zero();  // in the body frame
    if(pushAtCom_)
    {
      const auto & I = robot().mb().body(robot().bodyIndexByName(pushBody_)).inertia();
      pushPoint = I.momentum() / I.mass();
    }
    if(datastore().has(call))
    {
      datastore().call<bool, const std::string &, const sva::ForceVecd &, const Eigen::Vector3d>(
          call, pushBody_, sva::ForceVecd(Eigen::Vector3d::Zero(), pushForceVec_), pushPoint);
    }
  }
  else { pushForceVec_.setZero(); }
}

void G1RLQPController::updateAccelerationDiagnostics()
{
  const auto & mb = robot().mb();
  // QP output: the solver result is on outputRobot() (logged by mc_rtc as alphaDOut / ffOut_acceleration).
  qddQP_ = rbd::dofToVector(outputRobot().mb(), outputRobot().mbc().alphaD);

  auto & real_robot = realRobot(robot().name());
  alphaReal_ = rbd::dofToVector(real_robot.mb(), real_robot.mbc().alpha);

  if(rlmode != ACCELERATION || qddRefFull_.size() != mb.nrDof()) { return; }

  rbd::ForwardDynamics fd(real_robot.mb());
  fd.computeH(real_robot.mb(), real_robot.mbc());
  fd.computeC(real_robot.mb(), real_robot.mbc());
  const Eigen::MatrixXd & H = fd.H();
  Eigen::VectorXd tauExt = real_robot.externalTorques();
  if(tauExt.size() != mb.nrDof()) { tauExt = Eigen::VectorXd::Zero(mb.nrDof()); }

  const Eigen::Index nj = mb.nrDof() - 6;
  const Eigen::VectorXd rhs =
      H.block(0, 6, 6, nj) * qddRefFull_.tail(nj) + fd.C().head<6>() - tauExt.head<6>();
  qddBaseID_ = -H.topLeftCorner<6, 6>().ldlt().solve(rhs);

  // MJLab action law (see header), same filter as JointAccelerationAction.
  const double a = std::exp(-2.0 * M_PI * mjlabTauExtCutoffHz_ * timeStep);
  if(tauExtFiltered_.size() != mb.nrDof()) { tauExtFiltered_ = tauExt; }
  else { tauExtFiltered_ = a * tauExtFiltered_ + (1.0 - a) * tauExt; }
  Eigen::VectorXd qdd = qddRefFull_;
  qdd.head<6>() = -H.topLeftCorner<6, 6>().ldlt().solve(
      H.block(0, 6, 6, nj) * qdd.tail(nj) + fd.C().head<6>() - tauExtFiltered_.head<6>());
  const Eigen::VectorXd tauFull = H * qdd + fd.C() - tauExtFiltered_;
  tauMjlab_.resize(static_cast<Eigen::Index>(jointNames.size()));
  tauQP_.resize(tauMjlab_.size());
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    const auto jIdx = robot().jointIndexByName(jointNames[i]);
    const double lo = std::max(robot().tl()[jIdx][0], -150.0);
    const double hi = std::min(robot().tu()[jIdx][0], 150.0);
    const auto k = static_cast<Eigen::Index>(i);
    tauQP_(k) = robot().mbc().jointTorque[jIdx][0];
    tauMjlab_(k) = std::clamp(tauFull(mb.jointPosInDof(real_robot.jointIndexByName(jointNames[i]))), lo, hi);
    if(mjlabActionLaw_) { robot().mbc().jointTorque[jIdx][0] = tauMjlab_(k); }
  }
}

Eigen::VectorXd G1RLQPController::computeQaccExt()
{
  auto & real_robot = realRobot(robot().name());
  const auto & mb = real_robot.mb();
  const Eigen::Index nJoints = static_cast<Eigen::Index>(refJointOrderRLAction.size());
  Eigen::VectorXd out = Eigen::VectorXd::Zero(6 + nJoints);

  // Set by ExternalForcesObserver (update: true), in mc_rtc dof order.
  Eigen::VectorXd tauExt = real_robot.externalTorques();
  if(tauExt.size() != mb.nrDof()) { return out; }
  if(tauExtBaseForceWorld_)
  {
    // sva rotation is world-to-base.
    tauExt.segment<3>(3) = real_robot.mbc().bodyPosW[0].rotation() * tauExt.segment<3>(3);
  }

  // Same mass matrix as the observer (includes the rotor inertia, like MuJoCo's armature).
  rbd::ForwardDynamics fd(mb);
  fd.computeH(mb, real_robot.mbc());
  const Eigen::VectorXd qacc = fd.H().ldlt().solve(tauExt);

  // RBDyn free joint: [angular; linear] both in the base frame.
  // MuJoCo free joint: [linear in world frame; angular in base frame].
  const Eigen::Matrix3d R_world_base = real_robot.mbc().bodyPosW[0].rotation().transpose();
  out.segment<3>(0) = R_world_base * qacc.segment<3>(3);
  out.segment<3>(3) = qacc.segment<3>(0);

  for(Eigen::Index rlIndex = 0; rlIndex < nJoints; ++rlIndex)
  {
    const auto jointIndex = real_robot.jointIndexByName(refJointOrderRLAction[static_cast<size_t>(rlIndex)]);
    out(6 + rlIndex) = qacc(mb.jointPosInDof(static_cast<int>(jointIndex)));
  }
  return qaccExtGain_ * out;
}

// bool G1RLQPController::byPassQPControl()
// {
//   if(useQP_) return false; // QP is not bypassed, do nothing

//   robot().forwardKinematics();
//   robot().forwardVelocity();
//   robot().forwardAcceleration();

//   Eigen::VectorXd tau_rl_ = Eigen::VectorXd::Zero(nbActuatedJoints);
//   const auto & q_mbc = robot().mbc().q;
//   const auto & q_dot_mbc = robot().mbc().alpha;

//   int i = 0;
//   for(const auto & joint_name : jointNames)
//   {
//     const double q = q_mbc[robot().jointIndexByName(joint_name)][0];
//     const double q_dot = q_dot_mbc[robot().jointIndexByName(joint_name)][0];
//     tau_rl_(i) = kp_(i) * (q_rl(i) - q) - kd_(i) * q_dot;
//     robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl_(i);
//     i++;
//   }
//   return true;
// }

bool G1RLQPController::byPassQPControl()
{
  if(useQP_)
  {
    return false;
  }

  auto & measuredRobot = realRobot(robot().name());

  measuredRobot.forwardKinematics();
  measuredRobot.forwardVelocity();

  const auto & q_mbc = measuredRobot.mbc().q;
  const auto & q_dot_mbc = measuredRobot.mbc().alpha;

  const auto & tauLower = robot().tl();
  const auto & tauUpper = robot().tu();

  int saturatedJointCount = 0;
  double maxSaturationRatio = 0.0;

  int i = 0;

  for(const auto & jointName : jointNames)
  {
    const auto measuredJointIndex =
        measuredRobot.jointIndexByName(jointName);

    const auto commandJointIndex =
        robot().jointIndexByName(jointName);

    const double q =
        q_mbc[measuredJointIndex][0];

    const double qDot =
        q_dot_mbc[measuredJointIndex][0];

    const double tauRaw =
        kp_(i) * (q_rl(i) - q)
        - kd_(i) * qDot;

    const double tauMin =
        tauLower[commandJointIndex][0];

    const double tauMax =
        tauUpper[commandJointIndex][0];

    const double effortLimit =
        std::max(std::abs(tauMin), std::abs(tauMax));

    if(tauRaw < tauMin || tauRaw > tauMax)
    {
      saturatedJointCount++;
    }

    if(effortLimit > 1e-9)
    {
      maxSaturationRatio =
          std::max(
              maxSaturationRatio,
              std::abs(tauRaw) / effortLimit);
    }

    // const double tauCommand =
    //     std::max(tauMin, std::min(tauRaw, tauMax));

    const double tauCommand = tauRaw;

    robot().mbc().jointTorque[commandJointIndex][0] =
        tauCommand;

    i++;
  }

  static int debugCounter = 0;

  if(++debugCounter >= 400)
  {
    mc_rtc::log::info(
        "[RL DEBUG] torque saturation: {}/{} joints, max ratio={}",
        saturatedJointCount,
        jointNames.size(),
        maxSaturationRatio);

    debugCounter = 0;
  }

  return true;
}

std::pair<sva::PTransformd, Eigen::Vector3d> G1RLQPController::contactAnchor(const mc_rbdyn::Robot & r) const
{
  const auto & real = realRobot(robot().name());
  const Eigen::VectorXd & tauExt = real.externalTorques();
  auto kneeTorque = [&](const char * joint)
  {
    if(tauExt.size() != real.mb().nrDof()) { return 1.0; }
    return std::abs(tauExt(real.mb().jointPosInDof(real.jointIndexByName(joint))));
  };
  double wl = kneeTorque("left_knee_joint"), wr = kneeTorque("right_knee_joint");
  if(wl + wr < 1e-6) { wl = wr = 1.0; }
  const double leftRatio = wl / (wl + wr);
  auto solePoint = [&](const char * surface, Eigen::Vector3d & p, Eigen::Vector3d & v)
  {
    const auto & body = r.surface(surface).bodyName();
    const auto & velW = r.bodyVelW(body);
    p = r.surfacePose(surface).translation();
    v = velW.linear() + velW.angular().cross(p - r.bodyPosW(body).translation());
  };
  Eigen::Vector3d pl, vl, pr, vr;
  solePoint("LeftFootCenter", pl, vl);
  solePoint("RightFootCenter", pr, vr);
  return {sva::PTransformd(Eigen::Matrix3d::Identity(), leftRatio * pl + (1 - leftRatio) * pr),
          leftRatio * vl + (1 - leftRatio) * vr};
}

void G1RLQPController::addLog()
{
  // Base state the controller uses (estimated by the observer pipeline), to compare with FloatingBase_* (simulator).
  logger().addLogEntry("G1RLQPController_realBasePosW", [this]() { return realRobot(robot().name()).posW(); });
  logger().addLogEntry("G1RLQPController_realBaseVelW", [this]() { return realRobot(robot().name()).velW(); });
  logger().addLogEntry("G1RLQPController_qddQP", [this]() { return qddQP_; });
  logger().addLogEntry("G1RLQPController_alphaReal", [this]() { return alphaReal_; });
  logger().addLogEntry("G1RLQPController_qddBaseID", [this]() { return qddBaseID_; });
  logger().addLogEntry("G1RLQPController_tauMjlab", [this]() { return tauMjlab_; });
  logger().addLogEntry("G1RLQPController_tauQP", [this]() { return tauQP_; });
  logger().addLogEntry("G1RLQPController_velCmd", [this]() { return currentVelCmd; });
  logger().addLogEntry("G1RLQPController_pushForce", [this]() { return pushForceVec_; });

  logger().addLogEntry(
      "G1RLQPController_RL_currentObservation",
      [this]()
      {
        return currentObservation;
      });

  logger().addLogEntry(
      "G1RLQPController_RL_currentAction",
      [this]()
      {
        return currentAction;
      });

  logger().addLogEntry(
      "G1RLQPController_RL_actionScale",
      [this]()
      {
        return actionScale;
      });

  logger().addLogEntry(
      "G1RLQPController_qddRL",
      [this]()
      {
        return qddRL_;
      });

  logger().addLogEntry(
      "G1RLQPController_qddRefFull",
      [this]()
      {
        return qddRefFull_;
      });

  logger().addLogEntry(
      "G1RLQPController_accelerationActionRatio",
      [this]()
      {
        return accelerationActionRatio_;
      });

  logger().addLogEntry(
      "G1RLQPController_accelerationLimit",
      [this]()
      {
        return accelerationLimit_;
      });

  logger().addLogEntry(
      "G1RLQPController_postureRefAccel",
      [this]() -> Eigen::VectorXd
      {
        if(postureTaskRL_)
        {
          return postureTaskRL_->refAccel();
        }

        return Eigen::VectorXd::Zero(0);
      });

  logger().addLogEntry(
      "G1RLQPController_postureWeight",
      [this]()
      {
        return postureTaskRL_
                   ? postureTaskRL_->weight()
                   : 0.0;
      });

  logger().addLogEntry(
      "G1RLQPController_currentPolicy",
      [this]()
      {
        return std::to_string(currentPolicyIndex)
               + ": "
               + policyPaths_.at(currentPolicyIndex);
      });
}

void G1RLQPController::RLuseJoyStickInputs()
{
  // Get joystick functions
  auto & stickFunc = datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  // Read sticks values
  leftStick_ = stickFunc(joystickAnalogicInputs::L_STICK);
  // Apply dead zone
  double vel_x = 0.0;
  if(std::abs(leftStick_(0) - 0.5) > joystickDeadZone_)
  {
    vel_x = (leftStick_(0) - 0.5) * 2.0 * maxVelCmd_;
  }
  double vel_y = 0.0;
  if(std::abs(leftStick_(1) - 0.5) > joystickDeadZone_)
  {
    vel_y = (leftStick_(1) - 0.5) * 2.0 * maxVelCmd_;
  }
  currentVelCmd[0] = vel_x;
  currentVelCmd[1] = vel_y;

  rightStick_ = stickFunc(joystickAnalogicInputs::R_STICK);
  double yaw_cmd = 0.0;
  if(std::abs(rightStick_(1) - 0.5) > joystickDeadZone_)
  {
    yaw_cmd = (rightStick_(1) - 0.5) * 2.0 * maxYawCmd_;
  }
  currentVelCmd[2] = yaw_cmd;

  // Read D-pad buttons
  directionButtons_ = {datastore().get<bool>("Joystick::UpPad"), datastore().get<bool>("Joystick::DownPad"),
                       datastore().get<bool>("Joystick::LeftPad"), datastore().get<bool>("Joystick::RightPad")};

  for(size_t i = 0; i < directionButtons_.size(); ++i)
  {
    if(directionButtons_[i])
    {
      switch(i)
      {
        case 0: // Up
          currentVelCmd[0] += 1.0 * maxVelCmd_;
          break;
        case 1: // Down
          currentVelCmd[0] -= 1.0 * maxVelCmd_;
          break;
        case 2: // Left
          currentVelCmd[1] += 1.0 * maxVelCmd_;
          break;
        case 3: // Right
          currentVelCmd[1] -= 1.0 * maxVelCmd_;
          break;
        default:
          break;
      }
    }
  }
}

void G1RLQPController::addGui()
{
  gui()->addElement(
    {"G1RLQPController", "Acceleration Control"},

    mc_rtc::gui::NumberSlider(
        "Acceleration Action Ratio",
        [this]()
        {
          return accelerationActionRatio_;
        },
        [this](double value)
        {
          accelerationActionRatio_ = value;
        },
        0.0,
        1.0),

    mc_rtc::gui::Label(
        "Posture Kp",
        [this]()
        {
          return postureTaskRL_
                   ? postureTaskRL_->stiffness()
                   : -1.0;
        }),

    mc_rtc::gui::Label(
        "Posture Kd",
        [this]()
        {
          return postureTaskRL_
                   ? postureTaskRL_->damping()
                   : -1.0;
        }),

    mc_rtc::gui::Label(
        "Posture Weight",
        [this]()
        {
          return postureTaskRL_
                   ? postureTaskRL_->weight()
                   : -1.0;
        }));

  gui()->addElement(
      {"ControlMode"},
      mc_rtc::gui::Button(
          "Switch to acceleration policy",
          [this]() { resume("G1RLQPController_acceleration"); }),
      mc_rtc::gui::Button(
          "Switch to position policy",
          [this]() { resume("G1RLQPController_Initial"); }),
      mc_rtc::gui::Label(
          "Active RL mode",
          [this]() { return rlmode == ACCELERATION ? "Acceleration" : "Position"; }),
      mc_rtc::gui::Label("QP Control", [this]() { return useQP_ ? "Enforced" : "Bypassed"; }),
      mc_rtc::gui::Button("Toggle print joint limits", [this]() { printLimits_ = !printLimits_; }),
      mc_rtc::gui::Label("Print joint limits", [this]() { return printLimits_ ? "Enabled" : "Disabled"; }));

  // Example of changing the velocity command from the GUI, this should be adapted to the specific policy and
  // observation used. gui()->addElement({"G1RLQPController", "Velocity Command"},
  //     mc_rtc::gui::ArrayInput("Current Velocity Command", {"vx", "vy", "yaw_rate"}, currentVelCmd));
}

void G1RLQPController::configRL()
{
  mc_rtc::log::info("[G1RLQPController] Loading RL policy [{}]: {}", currentPolicyIndex,
                    policyPaths_[currentPolicyIndex]);
  if(!loadedPolicies_[currentPolicyIndex])
  {
    try
    {
      loadedPolicies_[currentPolicyIndex] =
          std::make_shared<RLPolicyInterface>(policyPaths_[currentPolicyIndex]);
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error_and_throw(
          "[G1RLQPController] Failed to load RL policy [{}]: {}", currentPolicyIndex, e.what());
    }
  }

  rlPolicy = loadedPolicies_[currentPolicyIndex];
  if(!rlPolicy || !rlPolicy->isLoaded())
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController] RL policy [{}] could not be loaded from {}",
        currentPolicyIndex,
        policyPaths_[currentPolicyIndex]);
  }

  mc_rtc::log::success(
      "[G1RLQPController] RL policy [{}] loaded: observation={}, action={}",
      currentPolicyIndex,
      rlPolicy->getObservationSize(),
      rlPolicy->getActionSize());

  const auto common = config_("policy_common")[0];
  const auto policy = config_("policies")[currentPolicyIndex];
  policyStepSize = policy("policy_step_size", common("policy_step_size", 0.01));
  const double physicsStepSize = policy("physics_step_size", common("physics_step_size", 0.0025));
  if(physicsStepSize - timeStep > 1e-6)
  {
    mc_rtc::log::warning("[G1RLQPController] Physics step size ({:.3f} s) is larger than controller time step ({:.3f} "
                         "s). This may cause issues with the policy. Consider fixing the controller time step.",
                         physicsStepSize, timeStep);
  }

  refJointOrderRLAction = common("ref_joint_order", std::vector<std::string>{});
  if(refJointOrderRLAction.size() != size_t(rlPolicy->getActionSize()))
  {
    mc_rtc::log::error_and_throw("[G1RLQPController] Reference joint order size ({}) does not match policy action "
                                 "size ({}). Please check the configuration.",
                                 refJointOrderRLAction.size(), rlPolicy->getActionSize());
  }

  // Create mapping from action indices to robot joint indices based on the reference joint order
  actionToDofMap.resize(refJointOrderRLAction.size(), -1); // Initialize with -1 to indicate unmapped actions
  for(size_t j = 0; j < refJointOrderRLAction.size(); ++j)
  {
    for(int i = 0; i < nbActuatedJoints; ++i)
    {
      if(jointNames[i] == refJointOrderRLAction[j])
      {
        actionToDofMap[j] = i;
        break;
      }
    }
  }

  // Create mapping between RL framework joint order and mc_rtc joint indices, this is useful for correctly ordering the
  // observation and action vectors according to the robot's joint order in mc_rtc.
  auto q0_map_cfg = common("q0");
  std::vector<std::string> keys = q0_map_cfg.keys(); // this preserves order

  if(keys.size() != static_cast<size_t>(nbActuatedJoints))
  {
    mc_rtc::log::error_and_throw("[G1RLQPController] The number of joints in q0 config ({}) does not match the "
                                 "robot's dof number ({}). Please check the configuration.",
                                 keys.size(), nbActuatedJoints);
  }

  // Compare if q0 order matches mc_rtc joint order, just to log it since the mapping will be created anyway.
  bool orderMatches = true;
  for(size_t i = 0; i < keys.size(); ++i)
  {
    if(keys[i] != jointNames[i])
    {
      orderMatches = false;
      break;
    }
  }
  if(orderMatches)
    mc_rtc::log::info(
        "[G1RLQPController] The order of joints in q0 config matches the robot's joint order in mc_rtc.");

  // rlFrameworkToMcRtcJointMap.resize(dofNumber, -1); // Initialize with -1 to indicate unmapped joints
  mcRtcToRLFrameworkJointMap.resize(nbActuatedJoints, -1);
  int j = 0;
  for(const auto & key : keys)
  {
    for(int i = 0; i < nbActuatedJoints; ++i)
    {
      if(jointNames[i] == key)
      {
        // rlFrameworkToMcRtcJointMap[j] = i;
        mcRtcToRLFrameworkJointMap[i] = j;
        break;
      }
    }
    j++;
  }

  for(int i = 0; i < nbActuatedJoints; ++i)
  {
    if(mcRtcToRLFrameworkJointMap[i] == -1)
    {
      mc_rtc::log::error_and_throw("[G1RLQPController] Joint '{}' was not properly mapped!", jointNames[i]);
    }
  }
}

void G1RLQPController::computeLimits()
{
  const double epsilon = 1e-5;

  auto & robot = robots()[0];
  const auto & currentPos = robot.q();
  const auto & currentVel = robot.alpha();
  const auto & currentTau = robot.jointTorque();

  const auto & qLimLower = robot.ql();
  const auto & qLimUpper = robot.qu();

  const auto & qDotLimLower = robot.vl();
  const auto & qDotLimUpper = robot.vu();

  const auto & tauLimLower = robot.tl();
  const auto & tauLimUpper = robot.tu();

  for(std::string joint : robot.refJointOrder())
  {
    size_t i = robot.jointIndexByName(joint);

    const double ds = dsPercent_ * (qLimUpper[i][0] - qLimLower[i][0]);
    const double posLimitUp = qLimUpper[i][0] - ds;
    const double posLimitLow = qLimLower[i][0] + ds;
    const double velLimitUp = velPercent_ * qDotLimUpper[i][0];
    const double velLimitLow = velPercent_ * qDotLimLower[i][0];
    const double tauLimitUp = tauLimUpper[i][0];
    const double tauLimitLow = tauLimLower[i][0];

    if(currentPos[i][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} position upper limit breached: currentPos = {}, limit = {}",
                           joint, currentPos[i][0], posLimitUp);
    }
    if(currentPos[i][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} position lower limit breached: currentPos = {}, limit = {}",
                           joint, currentPos[i][0], posLimitLow);
    }
    if(currentVel[i][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} velocity upper limit breached: currentVel = {}, limit = {}",
                           joint, currentVel[i][0], velLimitUp);
    }
    if(currentVel[i][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} velocity lower limit breached: currentVel = {}, limit = {}",
                           joint, currentVel[i][0], velLimitLow);
    }
    if(currentTau[i][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} torque upper limit breached: currentTau = {}, limit = {}",
                           joint, currentTau[i][0], tauLimitUp);
    }
    if(currentTau[i][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning("[G1RLQPController] Joint {} torque lower limit breached: currentTau = {}, limit = {}",
                           joint, currentTau[i][0], tauLimitLow);
    }
  }
}
