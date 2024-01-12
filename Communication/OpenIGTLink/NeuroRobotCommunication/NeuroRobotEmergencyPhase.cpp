#include "NeuroRobotEmergencyPhase.hpp"

NeuroRobotEmergencyPhase::NeuroRobotEmergencyPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {kStateNames.EMERGENCY, kStateNames.UNDEFINED};
}

NeuroRobotEmergencyPhase::~NeuroRobotEmergencyPhase()
{
}

void NeuroRobotEmergencyPhase::OnExit()
{
}

int NeuroRobotEmergencyPhase::Initialize()
{
  // Stop all motors and disable them.
  this->SendStatusMessage(this->Name(), 1, 0);
  robot->StopRobot();
  robot->motion_controller_.DisableAllMotors();
  return 1;
}

int NeuroRobotEmergencyPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
