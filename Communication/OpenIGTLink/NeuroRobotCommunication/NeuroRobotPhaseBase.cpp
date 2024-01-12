#include "NeuroRobotPhaseBase.hpp"

NeuroRobotPhaseBase::NeuroRobotPhaseBase()
{
  this->NextWorkphase.clear();
}

NeuroRobotPhaseBase::~NeuroRobotPhaseBase()
{
}

int NeuroRobotPhaseBase::Enter(const char *queryID)
{
  // Send acknowledgement message with query ID
  std::stringstream ss;
  std::string time(queryID);
  ss << kCommunicationCmds.ACKNOWLEDGE << queryID;
  this->SendStringMessage(ss.str().c_str(), this->Name());
  Logger &log = Logger::GetInstance();
  log.Log("Changed Workphase to " + string(this->Name()), ss.str().substr(4, std::string::npos), logger::INFO, 0);

  // Send phase message
  // TODO: Check if the phase transition is allowed
  this->SendStatusMessage(kMsgBody.CURRENT_STATUS.c_str(), 1, 0, this->Name());

  return this->Initialize();
}

int NeuroRobotPhaseBase::Process()
{
  // Create a message buffer to receive header
  igtl::MessageHeader::Pointer headerMsg;
  headerMsg = igtl::MessageHeader::New();

  ReceiveMessageHeader(headerMsg, 0);

  // If there is any workphase change request,
  // set NextWorkphase (done in the subroutine) and return 1.
  if (this->CheckWorkphaseChange(headerMsg))
  {
    return 1;
  }
  // If NOT empty message
  if (strcmp(headerMsg->GetDeviceName(), "") != 0)
  {
    this->NextWorkphase = this->Name();
  }

  // Common messages are handled here.
  if (!MessageHandler(headerMsg))
  {
    this->CheckCommonMessage(headerMsg);
  }
  return 0;
}

int NeuroRobotPhaseBase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}

int NeuroRobotPhaseBase::CheckWorkphaseChange(igtl::MessageHeader *headerMsg)
{

  // Check if the message requests phase transition
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0 &&
      strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.COMMAND.c_str(), 4) == 0)
  {
    igtl::StringMessage::Pointer stringMsg;
    stringMsg = igtl::StringMessage::New();
    stringMsg->SetMessageHeader(headerMsg);
    stringMsg->AllocatePack();
    bool timeout(false);

    // The code waits here for new messages recieved from the socket
    int r = this->Socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize(), timeout);
    if (r < 0)
    {
      std::cerr << "ERROR: Timeout." << std::endl;
      this->Socket->CloseSocket();
      return 0;
    }
    else if (r == 0)
    {
      std::cerr << "ERROR: Socket closed while reading a message." << std::endl;
      this->Socket->CloseSocket();
      return 0;
    }

    // Deserialize the string message
    // If you want to skip CRC check, call Unpack() without argument.
    int c = stringMsg->Unpack(1);

    if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
      if (stringMsg->GetEncoding() == 3)
      {
        this->NextWorkphase = stringMsg->GetString();
        // Get the query ID
        std::string msgName = headerMsg->GetDeviceName();
        this->QueryID = msgName.substr(4, std::string::npos);
        return 1;
      }
      else
      {
        this->NextWorkphase = "Unknown";
        return 1;
      }
    }
    else
    {
      std::cerr << "ERROR: Invalid CRC." << std::endl;
      this->NextWorkphase = "Unknown";
      return 1;
    }
  }
  else
  {
    return 0;
  }
}

// Checks for general communication messages that can be triggered from Slicer regardless of the current state.
int NeuroRobotPhaseBase::CheckCommonMessage(igtl::MessageHeader *headerMsg)
{
  // Check if the received message is string
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0)
  {
    string dev_name;
    ReceiveString(headerMsg, dev_name);

    if (strcmp(dev_name.c_str(), kMsgBody.CURRENT_POSITION.c_str()) == 0)
    {
      // Get the current position of the robot's needle tip
      igtl::Matrix4x4 currentPosition;
      RStatus->GetCurrentPosition(currentPosition);
      // Send navigation your current needle tip position
      SendTransformMessage(kMsgBody.CURRENT_POSITION.c_str(), currentPosition);
      // Send string msg flag for targeting position
      SendStringMessage(kMsgBody.IS_IN_TARGETING_POS.c_str(), to_string(robot->clinical_mode->isInTargetingPos()).c_str());
      // Send string msg flag for insertion depth
      SendStringMessage(kMsgBody.PROBE_IN_TREATMENT_REGION.c_str(), to_string(robot->clinical_mode->isProbeInplace()).c_str());
      return 1;
    }

    else if (strcmp(dev_name.c_str(), kMsgBody.CURRENT_STATUS.c_str()) == 0)
    {
      this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
  }

  else if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STATUS.c_str()) == 0)
  {
    int code;
    int subcode;
    std::string name;
    std::string status;
    ReceiveStatus(headerMsg, code, subcode, name, status);
    return 1;
  }
  return 0;
}

void NeuroRobotPhaseBase::OnExit()
{
  // specific behaviour should be defined in child classes
}

/* This method checks if the transition to a specific desired workphase is allowed and returns true for allowed transitions.*/
bool NeuroRobotPhaseBase::IsTransitionAllowed(const std::string &desired_transition)
{
  if (forbidden_transition_list.size() > 0)
  {
    for (auto state : forbidden_transition_list)
    {
      if (desired_transition == state)
      {
        Logger &log = Logger::GetInstance();
        string message = "Transition from " + string(this->Name()) + " to " + desired_transition + " is not allowed!";
        log.Log(message, logger::ERROR);
        return false;
      }
    }
  }
  return true;
};