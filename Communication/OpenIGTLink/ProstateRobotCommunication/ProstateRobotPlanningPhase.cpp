#include "ProstateRobotPlanningPhase.hpp"

ProstateRobotPlanningPhase::ProstateRobotPlanningPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.PLANNING,
      kStateNames.UNDEFINED,
      kStateNames.MOVE_TO_TARGET
  };
}

ProstateRobotPlanningPhase::~ProstateRobotPlanningPhase()
{
}

void ProstateRobotPlanningPhase::OnExit()
{
}

int ProstateRobotPlanningPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int ProstateRobotPlanningPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
