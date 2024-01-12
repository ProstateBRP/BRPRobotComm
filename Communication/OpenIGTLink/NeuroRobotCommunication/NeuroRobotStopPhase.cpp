#include "NeuroRobotStopPhase.hpp"

NeuroRobotStopPhase::NeuroRobotStopPhase() : NeuroRobotPhaseBase()
{
  // Don't let transition to states that involve robot motion unless the transition criteria in isTransitionAllowed function is satisfied.
  forbidden_transition_list = {
      kStateNames.STOP,
      kStateNames.UNDEFINED,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET
  };
}

NeuroRobotStopPhase::~NeuroRobotStopPhase()
{
}

void NeuroRobotStopPhase::OnExit()
{
}

int NeuroRobotStopPhase::Initialize()
{
  // Stop all motors and disable them.
  this->SendStatusMessage(this->Name(), 1, 0);
  robot->StopRobot();
  robot->motion_controller_.DisableAllMotors();

  return 1;
}

int NeuroRobotStopPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}

bool NeuroRobotStopPhase::IsTransitionAllowed(const std::string &desired_next_workphase)
{
  // If transitioned from move_to_target and transitioning back to move_to_target allow the transition.
  if (GetPreviousWorkPhase() == kStateNames.MOVE_TO_TARGET && desired_next_workphase == kStateNames.MOVE_TO_TARGET)
  {
    return true;
  }
  // If transitioned from targeting and transitioning back to targeting allow the transition.
  else if (GetPreviousWorkPhase() == kStateNames.TARGETING && desired_next_workphase == kStateNames.TARGETING)
  {
    return true;
  }
  return NeuroRobotStopPhase::IsTransitionAllowed(desired_next_workphase);
}