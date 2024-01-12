#include "ProstateRobotPhaseBase.hpp"

ProstateRobotPhaseBase::ProstateRobotPhaseBase() 
{
  this->NextWorkphase.clear();
}

ProstateRobotPhaseBase::~ProstateRobotPhaseBase()
{
}

int ProstateRobotPhaseBase::Enter(const char *queryID)
{
  // Send acknowledgement message with query ID
  std::stringstream ss;
  std::string time(queryID);
  ss << kCommunicationCmds.ACKNOWLEDGE << queryID;
  this->SendStringMessage(ss.str().c_str(), this->Name());
  Logger &log = Logger::GetInstance();
  log.Log("Changed Workphase to " + string(this->Name()), ss.str().substr(4, std::string::npos), logger::INFO, false);

  // Send phase message
  this->SendStatusMessage(kMsgBody.CURRENT_STATUS.c_str(), 1, 0, this->Name());

  return this->Initialize();
}

int ProstateRobotPhaseBase::Process()
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

int ProstateRobotPhaseBase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}

int ProstateRobotPhaseBase::CheckWorkphaseChange(igtl::MessageHeader *headerMsg)
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

    // The code waits here for new messages received from the socket
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
int ProstateRobotPhaseBase::CheckCommonMessage(igtl::MessageHeader *headerMsg)
{
  // Check if the received message is string
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0)
  {
    string string_msg_content;
    ReceiveString(headerMsg, string_msg_content);

    if (strcmp(headerMsg->GetDeviceName(), kMsgBody.CURRENT_POSITION.c_str()) == 0)
    {
      // Get the current position of the robot's needle tip
      igtl::Matrix4x4 currentPosition;
      RStatus->GetCurrentPosition(currentPosition);
      // Send navigation your current needle tip position
      SendTransformMessage(kMsgBody.CURRENT_POSITION.c_str(), currentPosition);
      // Send string msg flag for targeting position
      SendStringMessage(kMsgBody.IS_IN_TARGETING_POS.c_str(), to_string(robot->clinical_mode->isInTargetingPos()).c_str());
      // Send string msg flag for insertion depth
      SendStringMessage(kMsgBody.HAS_REACHED_TARGET.c_str(), to_string(robot->clinical_mode->hasReachedTarget()).c_str());
      return 1;
    }

    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.CURRENT_STATUS.c_str()) == 0)
    {
      this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
    
    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.SET_DEADBAND.c_str()) == 0)
    {
      try
      {
        int deadband = std::stoi(string_msg_content);
        RStatus->SetDeadband(deadband);
        Logger &log = Logger::GetInstance();
        log.Log("Deadband of " + string_msg_content + " was received and set in the code.", logger::INFO, true);
      }
      catch(const std::invalid_argument &e)
      {
        std::cerr << "Bad deadband value. Message ignored!" << std::endl;
      }
      return 1;
    }

    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.ALPHA.c_str()) == 0)
    {
      try
      {
        double alpha_d = std::stod(string_msg_content);
        RStatus->SetAlpha(alpha_d);
      }
      catch(const std::invalid_argument&e)
      {
        std::cerr << e.what() << std::endl;
      }
      return 1;
    }
    
    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.TARGET_ANGLE.c_str()) == 0)
    {
      try
      {
        double theta_d = std::stod(string_msg_content);
        RStatus->SetTargetAngle(theta_d);
      }
      catch(const std::invalid_argument&e)
      {
        std::cerr << e.what() << std::endl;
      }
      return 1;
    }

    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.CURV_METHOD.c_str()) == 0)
    {
      try
      {
        this->RStatus->SetCurvMethod(string_msg_content);
        SendStatusMessage(kMsgBody.CURV_METHOD.c_str(), igtl::StatusMessage::STATUS_OK, 0);
      }
      catch(const std::invalid_argument&e)
      {
        std::cerr << e.what() << std::endl;
        SendStatusMessage(kMsgBody.CURV_METHOD.c_str(), igtl::StatusMessage::STATUS_INVALID, 0);
      }
      // Send acknowledgment for successful needle retraction.
      return 1;
    }
    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.SET_STEERING.c_str()) == 0)
    {
      try
      {
        RStatus->SetSteeringMethod(string_msg_content);
      }
      catch(const std::invalid_argument &e)
      {
        std::cerr << e.what() << std::endl;
      }
      return 1;
    }
    // Catch all undefined string messages
    else
    {
      std::string msg_name = headerMsg->GetDeviceName();
      // Acknowledge receipt of the undefined message with INVALID status
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE.c_str() << msg_name.c_str();
      SendStatusMessage(ss.str().c_str(), igtl::StatusMessage::STATUS_INVALID, 0);
      Logger &log = Logger::GetInstance();
      string message = "STRING message: " + msg_name + " was received but was not expected. Message is discarded.";
      log.Log(message, logger::WARNING, true);

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
  else if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.TRANSFORM.c_str()) == 0)
  {
    igtl::Matrix4x4 transform;
    // Update the base to zframe
      Logger &log = Logger::GetInstance();
    string message = "TRANSFORM message: " + string(headerMsg->GetDeviceName()) + " was received but was not expected. Message is discarded.";
    log.Log(message, logger::WARNING, true);
    ReceiveTransform(headerMsg,transform);
    return 1;
  }
  return 0;
}

void ProstateRobotPhaseBase::OnExit()
{
  // specific behaviour should be defined in child classes
}

/* This method checks if the transition to a specific desired workphase is allowed and returns true for allowed transitions.*/
bool ProstateRobotPhaseBase::IsTransitionAllowed(const std::string &desired_transition)
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

  return true;
};
