#include "NeuroRobotDrapingPhase.hpp"

NeuroRobotDrapingPhase::NeuroRobotDrapingPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.DRAPING,
      kStateNames.ABLATION,
      kStateNames.MOVE_TO_TARGET,
      kStateNames.UNDEFINED,
  };
}

NeuroRobotDrapingPhase::~NeuroRobotDrapingPhase()
{
}

int NeuroRobotDrapingPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  robot->clinical_mode->SetMotorSetpointsToDraping();
  return 1;
}

void NeuroRobotDrapingPhase::OnExit()
{
}

int NeuroRobotDrapingPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
