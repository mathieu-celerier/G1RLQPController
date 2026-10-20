#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <mc_rtc/logging.h>

#include "G1RLQPController.h"

void utils::start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  state_name_ = state_name;
  mc_rtc::log::info("[G1RLQPController::utils] {} state started", state_name);

  syncTime_ = ctl.policyStepSize;
    
  if(!ctl.rlPolicy || !ctl.rlPolicy->isLoaded())
  {
    mc_rtc::log::error("[G1RLQPController::utils] RL policy not loaded in {} state", state_name);
    return;
  }

  ctl.currentAction.setZero();
  ctl.currentActionScaled.setZero();
  ctl.q_rl = ctl.q_zero;
  ctl.resetRLHistory();

  mc_rtc::log::success("[G1RLQPController::utils] {} state initialization completed", state_name);
}

void utils::run_rl_state(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  try
  {
    syncTime_ += ctl.timeStep;
    if(syncTime_ >= ctl.policyStepSize)
    {
      ctl.currentObservation = getCurrentObservation(ctl);
      ctl.currentAction = ctl.rlPolicy->predict(ctl.currentObservation);

      if(ctl.currentAction.size() != static_cast<Eigen::Index>(ctl.actionToDofMap.size())
         || !ctl.currentAction.allFinite())
      {
        mc_rtc::log::error_and_throw(
            "[G1RLQPController::utils] Invalid policy action: size={}, expected={}, finite={}",
            ctl.currentAction.size(),
            ctl.actionToDofMap.size(),
            ctl.currentAction.allFinite());
      }

      if(ctl.rlmode == ACCELERATION)
      {
        if(!ctl.postureTaskRL_)
        {
          mc_rtc::log::error_and_throw(
              "[G1RLQPController::utils] Cannot apply acceleration action: PostureTask is null");
        }

        ctl.qddRL_.setZero();
        ctl.qddRefFull_.setZero();

        for(Eigen::Index j = 0; j < ctl.currentAction.size(); ++j)
        {
          const size_t actionIndex = static_cast<size_t>(j);
          const int actuatedIndex = ctl.actionToDofMap.at(actionIndex);

          if(actuatedIndex < 0)
          {
            mc_rtc::log::error_and_throw(
                "[G1RLQPController::utils] Action {} is not mapped to a robot joint", actionIndex);
          }

          const std::string & jointName = ctl.refJointOrderRLAction.at(actionIndex);
          const int jointIndex = ctl.robot().jointIndexByName(jointName);

          if(ctl.robot().mb().joint(jointIndex).dof() != 1)
          {
            mc_rtc::log::error_and_throw(
                "[G1RLQPController::utils] Joint {} must have exactly one DoF", jointName);
          }

          const int fullDofIndex = ctl.robot().mb().jointPosInDof(jointIndex);
          const double qddRaw =
              ctl.accelerationActionRatio_ * ctl.actionScale(actuatedIndex) * ctl.currentAction(j);
          const double qdd = std::clamp(qddRaw, -ctl.accelerationLimit_, ctl.accelerationLimit_);

          // PostureTask/TVM expects the canonical mc_rtc actuated-joint order.
          ctl.qddRL_(actuatedIndex) = qdd;
          ctl.qddRefFull_(fullDofIndex) = qdd;
        }

        ctl.postureTaskRL_->refAccel(ctl.qddRL_);
      }
      else if(ctl.rlmode == POSITION)
      {
        if(!ctl.torqueJointTaskRL_)
        {
          mc_rtc::log::error_and_throw(
              "[G1RLQPController::utils] Cannot apply position action: TorqueJointTask is null");
        }

        ctl.qddRL_.setZero();
        ctl.qddRefFull_.setZero();

        ctl.currentActionScaled.setZero();
        ctl.q_rl = ctl.q_zero;
        for(Eigen::Index j = 0; j < ctl.currentAction.size(); ++j)
        {
          const size_t actionIndex = static_cast<size_t>(j);
          const int i = ctl.actionToDofMap.at(actionIndex);

          if(i < 0)
          {
            mc_rtc::log::error_and_throw(
                "[G1RLQPController::utils] Action {} is not mapped to a robot joint", actionIndex);
          }

          ctl.currentActionScaled(i) = ctl.actionScale(i) * ctl.currentAction(j);
          ctl.q_rl(i) = ctl.currentActionScaled(i) + ctl.q_zero(i);
        }

        ctl.torqueJointTaskRL_->setPosTarget(ctl.q_rl);
      }
      else
      {
        mc_rtc::log::error_and_throw(
            "[G1RLQPController::utils] Unsupported RL mode: {}", ctl.rlmode);
      }

      syncTime_ -= ctl.policyStepSize;
      if(syncTime_ < 0.0) { syncTime_ = 0.0; }
    }
  }
  catch(const std::exception & e)
  {
    ctl.currentAction.setZero();
    ctl.currentActionScaled.setZero();
    ctl.qddRL_.setZero();
    ctl.qddRefFull_.setZero();

    if(ctl.rlmode == POSITION && ctl.torqueJointTaskRL_)
    {
      ctl.q_rl = ctl.q_zero;
      ctl.torqueJointTaskRL_->setPosTarget(ctl.q_rl);
    }
    else if(ctl.postureTaskRL_)
    {
      ctl.postureTaskRL_->refAccel(ctl.qddRL_);
    }

    mc_rtc::log::error("[G1RLQPController::utils] Error during RL state run: {}", e.what());
  }
}

Eigen::VectorXd utils::getCurrentObservation(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  Eigen::VectorXd obs = Eigen::VectorXd::Zero(ctl.rlPolicy->getObservationSize());

  int offset = 0;
  auto appendToObs = [&](const Eigen::VectorXd & v)
  {
    if(offset + v.size() > obs.size())
    {
      mc_rtc::log::error_and_throw(
          "[G1RLQPController::utils] Observation overflow: offset={}, term size={}, policy size={}",
          offset,
          v.size(),
          obs.size());
    }

    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  // Both policies were trained with the same seven observation terms and the
  // same five-sample history. Index 0 is newest, index 4 oldest.
  for(int i = ctl.HISTORY_SIZE - 1; i > 0; --i)
  {
    ctl.linVel[i] = ctl.linVel[i - 1];
    ctl.angVel[i] = ctl.angVel[i - 1];
    ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
    ctl.jointPos[i] = ctl.jointPos[i - 1];
    ctl.jointVel[i] = ctl.jointVel[i - 1];
    ctl.jointAction[i] = ctl.jointAction[i - 1];
    ctl.velCmd[i] = ctl.velCmd[i - 1];
  }

  // currentAction is still the previous raw action here, as in MJLab.
  ctl.initializeRLObservation();

  // MJLab actor term order; each history is stacked oldest to newest.
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.linVel[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.angVel[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.projectedGravity[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.jointPos[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.jointVel[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.jointAction[i]); }
  for(int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) { appendToObs(ctl.velCmd[i]); }

  if(offset != obs.size())
  {
    mc_rtc::log::error_and_throw(
        "[G1RLQPController::utils] Observation size mismatch: wrote {}, policy expects {}", offset, obs.size());
  }

  if(!obs.allFinite())
  {
    mc_rtc::log::error_and_throw("[G1RLQPController::utils] Observation contains NaN or Inf");
  }

  return obs;
}
