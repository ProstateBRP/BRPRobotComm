#include "ProstateRobotManualPhase.hpp"

ProstateRobotManualPhase::ProstateRobotManualPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.MANUAL,
      kStateNames.UNDEFINED,
      kStateNames.CALIBRATION,
      kStateNames.MOVE_TO_TARGET,
      kStateNames.PLANNING,
      kStateNames.TARGETING
  };
}

ProstateRobotManualPhase::~ProstateRobotManualPhase()
{
}

void ProstateRobotManualPhase::OnExit()
{
}

int ProstateRobotManualPhase::Initialize()
{

  // Send Status after waiting for 2 seconds (mimicking initialization process)
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int ProstateRobotManualPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
