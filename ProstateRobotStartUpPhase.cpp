#include "ProstateRobotStartUpPhase.hpp"

ProstateRobotStartUpPhase::ProstateRobotStartUpPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.START_UP,
      kStateNames.UNDEFINED,
      kStateNames.PLANNING,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET
  };
}

ProstateRobotStartUpPhase::~ProstateRobotStartUpPhase()
{
}

void ProstateRobotStartUpPhase::OnExit()
{
}

int ProstateRobotStartUpPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

int ProstateRobotStartUpPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  // Receive and set needle length
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0 &&
      strcmp(headerMsg->GetDeviceName(), kMsgBody.NEEDLE_LENGTH.c_str()) == 0)
  {
    // Send acknowledgment for retract needle command
    string needle_length;
    ReceiveString(headerMsg, needle_length);
    this->RStatus->SetNeedleLength(std::stod(needle_length));
    std::stringstream ss;
    ss << kCommunicationCmds.ACKNOWLEDGE << kMsgBody.NEEDLE_LENGTH;
    SendStringMessage(ss.str().c_str(), needle_length.c_str());
    return 1;
  }
  return 0;
}
