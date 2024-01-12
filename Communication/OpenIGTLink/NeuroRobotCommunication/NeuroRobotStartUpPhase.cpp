#include "NeuroRobotStartUpPhase.hpp"

NeuroRobotStartUpPhase::NeuroRobotStartUpPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.START_UP,
      kStateNames.UNDEFINED,
      kStateNames.PLANNING,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET,
      kStateNames.DRAPING,
      kStateNames.ABLATION
  };
}

NeuroRobotStartUpPhase::~NeuroRobotStartUpPhase()
{
}

void NeuroRobotStartUpPhase::OnExit()
{
}

int NeuroRobotStartUpPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), 1, 0);
  // Prepare robot for homing
  return 1;
}

int NeuroRobotStartUpPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  // Perform homing of the robot
  return 0;
}

bool NeuroRobotStartUpPhase::IsTransitionAllowed(const std::string &desired_phase)
{
  if (desired_phase == kStateNames.DRAPING && robot->isHomed())
  {
    return true;
  }
  return NeuroRobotPhaseBase::IsTransitionAllowed(desired_phase);
}
