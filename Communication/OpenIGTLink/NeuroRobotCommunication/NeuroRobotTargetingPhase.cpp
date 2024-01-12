#include "NeuroRobotTargetingPhase.hpp"

NeuroRobotTargetingPhase::NeuroRobotTargetingPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {kStateNames.TARGETING, kStateNames.UNDEFINED};
}

NeuroRobotTargetingPhase::~NeuroRobotTargetingPhase()
{
}

void NeuroRobotTargetingPhase::OnExit()
{
  // Stop all motors and disable them upon exit
  robot->StopRobot();
  robot->motion_controller_.DisableAllMotors();
}

int NeuroRobotTargetingPhase::Initialize()
{
  // If the robot has not been calibrated, return device-not-ready error
  Logger &log = Logger::GetInstance();

  if (!this->RStatus || !this->RStatus->GetCalibrationFlag())
  {
    std::cerr << "ERROR: Attempting to start TARGETING without CALIBRATION." << std::endl;
    log.Log("Attempting to start TARGETING without CALIBRATION.", logger::ERROR, true);

    this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_ACCESS_DENIED, 0);
  }
  else
  {
    this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  }

  return 1;
}

int NeuroRobotTargetingPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  Logger &log = Logger::GetInstance();
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.TRANSFORM.c_str()) == 0)
  {
    // Check if EntryPoint has been received
    if (strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.ENTRY.c_str(), 4) == 0)
    {
      // Creating a 4x4 matrix to receive the incoming entry point
      igtl::Matrix4x4 matrix;
      this->ReceiveTransform(headerMsg, matrix);

      // Sending an acknowledgement transform back to the navigation
      std::string devName = headerMsg->GetDeviceName();
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE.c_str() << devName.substr(4, std::string::npos);
      SendTransformMessage(ss.str().c_str(), matrix);
      log.Log("OpenIGTLink Entry Point Received", devName.substr(4, std::string::npos), logger::INFO, true);

      // Validation process for the received Entry point
      // This should include the IK check and the general validation test.
      if (ValidateMatrix(matrix))
      {
        // Set the entry pose in RAS for the robot
        this->RStatus->SetEntryMatrix(matrix);
        log.Log("OpenIGTLink Entry Point validated and Set in Code", devName.substr(4, std::string::npos),
                logger::INFO, true);
        log.Log(matrix, kMsgBody.ENTRY, devName.substr(4, std::string::npos), logger::INFO, true);
        // Reporting back the reachable target pose
        igtl::Matrix4x4 reachable_target;
        RStatus->GetReachableTargetPoseMatrix(reachable_target);
        // Send an Status message-- OK
        SendStatusMessage(kMsgBody.ENTRY.c_str(), igtl::StatusMessage::STATUS_OK, 0);
        // This will update the navigation regarding the reachable target
        SendTransformMessage(kMsgBody.REACHABLE_TARGET.c_str(), reachable_target);
        log.Log(reachable_target, kMsgBody.REACHABLE_TARGET, devName.substr(4, std::string::npos), logger::INFO, true);
      }
      else
      {
        std::cerr << "ERROR: Invalid Entry Point matrix." << std::endl;
        // Send an Status message-- ERROR
        SendStatusMessage(kMsgBody.ENTRY.c_str(), igtl::StatusMessage::STATUS_CONFIG_ERROR, 0);
      }
    }
    // Check if TargetPoint has been received
    else if (strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.TARGET.c_str(), 4) == 0)
    {
      // Creating a 4x4 matrix to receive the incoming entry point
      igtl::Matrix4x4 matrix;
      this->ReceiveTransform(headerMsg, matrix);
      // Sending an acknowledgement transform back to the navigation
      std::string devName = headerMsg->GetDeviceName();
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE.c_str() << devName.substr(4, std::string::npos);
      SendTransformMessage(ss.str().c_str(), matrix);
      log.Log("OpenIGTLink Target Point Received", devName.substr(4, std::string::npos), logger::INFO, true);

      // Validation process for the received Target point
      if (ValidateMatrix(matrix))
      {
        log.Log("OpenIGTLink Target Point validated and Set in Code", devName.substr(4, std::string::npos),
                logger::INFO, true);
        log.Log(matrix, kMsgBody.TARGET, devName.substr(4, std::string::npos), logger::INFO, false);

        // Set the target pose in RAS for the robot
        this->RStatus->SetTargetMatrix(matrix);
        // Reporting back the reachable target pose
        igtl::Matrix4x4 reachable_target;
        RStatus->GetReachableTargetPoseMatrix(reachable_target);
        // Check if reachable target is equal to received target
        if (CompareMatrices(reachable_target, matrix, 0.001))
        {
          // Send an Status message-- OK
          SendStatusMessage(kMsgBody.TARGET.c_str(), igtl::StatusMessage::STATUS_OK, 0);
        }
        else
        {
          // Send an Status message-- config error
          SendStatusMessage(kMsgBody.TARGET.c_str(), igtl::StatusMessage::STATUS_CONFIG_ERROR, 3);
        }
        // This will update the navigation regarding the reachable target
        SendTransformMessage(kMsgBody.REACHABLE_TARGET.c_str(), reachable_target);
        log.Log(reachable_target, kMsgBody.REACHABLE_TARGET, devName.substr(4, std::string::npos), logger::INFO, false);
      }
      else
      {
        // Send an Status message-- ERROR
        std::cerr << "ERROR: Invalid Target Point matrix." << std::endl;
        SendStatusMessage(kMsgBody.TARGET.c_str(), igtl::StatusMessage::STATUS_CONFIG_ERROR, 1);
      }
    }
    return 1;
  }
  return 0;
}
