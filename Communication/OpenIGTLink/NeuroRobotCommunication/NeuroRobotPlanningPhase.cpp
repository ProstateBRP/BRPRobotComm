#include "NeuroRobotPlanningPhase.hpp"

NeuroRobotPlanningPhase::NeuroRobotPlanningPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.PLANNING,
      kStateNames.UNDEFINED,
      kStateNames.MOVE_TO_TARGET,
      kStateNames.ABLATION
  };
}

NeuroRobotPlanningPhase::~NeuroRobotPlanningPhase()
{
}

void NeuroRobotPlanningPhase::OnExit()
{
}

int NeuroRobotPlanningPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int NeuroRobotPlanningPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  // TODO: Set probe specifications in here
  return 0;
}
