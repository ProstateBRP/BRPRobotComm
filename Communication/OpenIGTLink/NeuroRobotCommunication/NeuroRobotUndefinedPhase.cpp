#include "NeuroRobotUndefinedPhase.hpp"

NeuroRobotUndefinedPhase::NeuroRobotUndefinedPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.UNDEFINED,
      kStateNames.CALIBRATION,
      kStateNames.MANUAL,
      kStateNames.DRAPING,
      kStateNames.PLANNING,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET};

}

void NeuroRobotUndefinedPhase::OnExit()
{
}

NeuroRobotUndefinedPhase::~NeuroRobotUndefinedPhase()
{
}

int NeuroRobotUndefinedPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int NeuroRobotUndefinedPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
