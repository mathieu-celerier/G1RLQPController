#include "G1RLQPController_acceleration.h"

#include "../G1RLQPController.h"

void G1RLQPController_acceleration::configure(const mc_rtc::Configuration & config) {}

void G1RLQPController_acceleration::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.enterAccelerationControl();
  ctl.utilsClass.start_rl_state(ctl, "Acceleration policy");
}

bool G1RLQPController_acceleration::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  
  return false;
}

void G1RLQPController_acceleration::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.leaveAccelerationControl();
}

EXPORT_SINGLE_STATE("G1RLQPController_acceleration", G1RLQPController_acceleration)
