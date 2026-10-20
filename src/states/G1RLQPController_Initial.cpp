#include "G1RLQPController_Initial.h"

#include "../G1RLQPController.h"

void G1RLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void G1RLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.enterPositionControl();
  ctl.utilsClass.start_rl_state(ctl, "Position policy");
}

bool G1RLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  return false;
}

void G1RLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<G1RLQPController &>(ctl_);
  ctl.leavePositionControl();
}

EXPORT_SINGLE_STATE("G1RLQPController_Initial", G1RLQPController_Initial)
