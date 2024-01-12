#include "ProstateRobotMoveToTargetPhase.hpp"

ProstateRobotMoveToTargetPhase::ProstateRobotMoveToTargetPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {kStateNames.UNDEFINED};
}

ProstateRobotMoveToTargetPhase::~ProstateRobotMoveToTargetPhase()
{
}

// TODO Prevent transition from all states to Move to target state other than from the targeting.
//   https://wpiaimlab.atlassian.net/browse/MRC-52
int ProstateRobotMoveToTargetPhase::Initialize()
{
  // If robot is in the targeting position
  if (robot->clinical_mode->isInTargetingPos())
  {
    SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
    // If we did not transition from a Stop state, initialize kinematic tip estimation.
    if (GetPreviousWorkPhase() == kStateNames.TARGETING)
    {
      // Save the current needle guide position as the first actual needle tip positions.
      robot->clinical_mode->PushBackKinematicTipAsActualPose();
      // Calculate Steering Effort
      robot->clinical_mode->UpdateCurvParamsAndInsertionLength();
    }
  }
  else
  {
    // Robot is not in the targeting position
    SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_NOT_READY, 0);
  }
  // Send current needle pose to Slicer
  igtl::Matrix4x4 current_pos;
  this->RStatus->GetCurrentKinematicTipPosition(current_pos);
  SendTransformMessage(kMsgBody.CURRENT_POSITION.c_str(), current_pos);
  return 1;
}

void ProstateRobotMoveToTargetPhase::OnExit()
{
  // Stop all motors and disable them upon exit
  robot->StopRobot();
  robot->motion_controller_.DisableAllMotors();
  // Clean up the reported tip positions from the buffer (Skip process if robot is Stopped).
  if (GetNextWorkPhase() != kStateNames.STOP)
  {
    robot->clinical_mode->CleanUp();
  }
}

int ProstateRobotMoveToTargetPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  Logger &log = Logger::GetInstance();
  // Check String messages
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0)
  {
    if (strcmp(headerMsg->GetDeviceName(), kMsgBody.CURRENT_POSITION.c_str()) == 0)
    {
      string text;
      ReceiveString(headerMsg, text);
      // Get the current position of the robot's needle tip
      igtl::Matrix4x4 currentPosition;
      // If retracting the needle report the rigid kinematics
      if (robot->clinical_mode->isRetractingNeedle())
      {
        RStatus->GetCurrentPosition(currentPosition);
      }
      // Report nonholonomic unicycle kinematics if inserting 
      else
      {
        RStatus->GetCurrentKinematicTipPosition(currentPosition);
      }
      // Send navigation your current needle tip position
      SendTransformMessage(kMsgBody.CURRENT_POSITION.c_str(), currentPosition);
      // Send string msg flag for targeting position
      SendStringMessage(kMsgBody.IS_IN_TARGETING_POS.c_str(), to_string(robot->clinical_mode->isInTargetingPos()).c_str());
      // Send string msg flag for insertion depth
      SendStringMessage(kMsgBody.HAS_REACHED_TARGET.c_str(), to_string(robot->clinical_mode->hasReachedTarget()).c_str());
      return 1;
    }
    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.RETRACT_NEEDLE.c_str()) == 0)
    {
      string text;
      ReceiveString(headerMsg, text);
      log.Log("Received Needle Retract Command From the Planning Software.",logger::INFO);
      // Send acknowledgment for retract needle command
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE << kMsgBody.RETRACT_NEEDLE;
      SendStatusMessage(ss.str().c_str(), igtl::StatusMessage::STATUS_OK, 0);
      // Robot is ready to retract to home upon pressing of the foot pedal.
      this->RStatus->RetractNeedleToHome();
      // Send acknowledgment for successful needle retraction.
      SendStatusMessage(kMsgBody.RETRACT_NEEDLE.c_str(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
    // This is for the development purposes
    else if (strcmp(headerMsg->GetDeviceName(), kMsgBody.CURV_METHOD.c_str()) == 0)
    {
      string text;
      ReceiveString(headerMsg, text);
      // Robot is ready to retract to home upon pressing of the foot pedal.
      this->RStatus->SetCurvMethod(text);
      // Send acknowledgment for successful needle retraction.
      SendStatusMessage(kMsgBody.CURV_METHOD.c_str(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
  }
  /// Imager transforms
  else if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.TRANSFORM.c_str()) == 0)
  {
    if (strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.TARGET.c_str(), 4) == 0)
    {
      // Creating a 4x4 matrix to receive the incoming entry point
      igtl::Matrix4x4 matrix;
      this->ReceiveTransform(headerMsg, matrix);

      // Sending an acknowledgement transform back to the navigation
      std::string devName = headerMsg->GetDeviceName();
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE.c_str() << devName.substr(4, std::string::npos);
      SendTransformMessage(ss.str().c_str(), matrix);
      log.Log("OpenIGTLink New Target Received", devName.substr(4, std::string::npos), logger::INFO, true);
      log.Log(matrix, kMsgBody.TARGET, devName.substr(4, std::string::npos), logger::INFO, false);

      // Set the target pose in RAS for the robot
      this->RStatus->UpdateTargetMatrix(matrix);
      // Update CURV parameters,
      // TODO: inform slicer if the target exceeds natural curvature
      this->robot->clinical_mode->UpdateCurvParamsAndInsertionLength();

      log.Log("OpenIGTLink Target Point Validated and CURV Parameters Updated!", devName.substr(4, std::string::npos),
              logger::INFO, true);
      std::string log_contents = "Alpha Set To: " + std::to_string(robot->clinical_mode->GetAlpha()) + ", Target Location: " + std::to_string(robot->clinical_mode->GetTargetAngle() * 180/M_PI) + " degrees.";
      log.Log(log_contents, devName.substr(4, std::string::npos), logger::INFO, true);

      return 1;
    }
    // Check if the navigation is sending the needle tip position
    else if (strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.NEEDLE_POS.c_str(), 5) == 0)
    {
      // Create a matrix to store needle pose
      std::string devName = headerMsg->GetDeviceName();
      igtl::Matrix4x4 matrix;
      this->ReceiveTransform(headerMsg, matrix);

      // Acknowledgement
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE.c_str() << devName.substr(5, std::string::npos);
      SendTransformMessage(ss.str().c_str(), matrix);

      Logger &log = Logger::GetInstance();
      log.Log("OpenIGTLink Needle Tip Received and Set in Code.", devName.substr(5, std::string::npos), logger::INFO, true);
      log.Log(matrix, "Needle Position", devName.substr(4, std::string::npos), logger::INFO, true);

      // Pushback the reported needle tip position and estimate actual tip pose
      RStatus->PushBackActualNeedlePos(matrix);
      log.Log("OpenIGTLink Needle Tip Received and Set in Code.", devName.substr(5, std::string::npos), logger::INFO, true);
      SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);

      // TODO: Warn Slicer if the new pose causes the target to be out of reach.
      SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
  }
  return 0;
}

bool ProstateRobotMoveToTargetPhase::IsTransitionAllowed(const std::string &desired_next_workphase)
{
  // If it's an Emergency OR Stop transition allow it regardless of the needle condition.
  if ((desired_next_workphase == kStateNames.EMERGENCY) || (desired_next_workphase == kStateNames.STOP))
  {
    return true;
  }
  // Don't allow transition if the needle is being retracted OR not at home.
  if ((robot->clinical_mode->isRetractingNeedle()) || (!robot->clinical_mode->isNeedleAtHome()))
  {
    return false;
  }
  return ProstateRobotPhaseBase::IsTransitionAllowed(desired_next_workphase);
}
