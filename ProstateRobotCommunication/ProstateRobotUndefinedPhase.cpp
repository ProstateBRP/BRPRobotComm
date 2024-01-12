#include "ProstateRobotUndefinedPhase.hpp"

ProstateRobotUndefinedPhase::ProstateRobotUndefinedPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.UNDEFINED,
      kStateNames.CALIBRATION,
      kStateNames.MANUAL,
      kStateNames.PLANNING,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET};
      
}

void ProstateRobotUndefinedPhase::OnExit()
{
}

ProstateRobotUndefinedPhase::~ProstateRobotUndefinedPhase()
{
}

int ProstateRobotUndefinedPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int ProstateRobotUndefinedPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
