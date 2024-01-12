#include "NeuroRobotMoveToTargetPhase.hpp"

NeuroRobotMoveToTargetPhase::NeuroRobotMoveToTargetPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {kStateNames.MOVE_TO_TARGET, kStateNames.UNDEFINED};
}

NeuroRobotMoveToTargetPhase::~NeuroRobotMoveToTargetPhase()
{
}

// TODO: Prevent transition from all states to Move to target state other than from the targeting.
int NeuroRobotMoveToTargetPhase::Initialize()
{
  // If robot is in the targeting position
  if (robot->clinical_mode->isInTargetingPos())
  {
    SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  }
  else
  {
    // Robot is not in the targeting position
    SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_NOT_READY, 0);
  }
  // Send current needle pose to Slicer
  igtl::Matrix4x4 current_pos;
  this->RStatus->GetCurrentPosition(current_pos);
  SendTransformMessage(kMsgBody.CURRENT_POSITION.c_str(), current_pos);
  return 1;
}

void NeuroRobotMoveToTargetPhase::OnExit()
{
  // Stop all motors and disable them upon exit
  robot->StopRobot();
  robot->motion_controller_.DisableAllMotors();
}

int NeuroRobotMoveToTargetPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  Logger &log = Logger::GetInstance();
  // Check if retract probe command is received
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.STRING.c_str()) == 0)
  {
    string text;
    ReceiveString(headerMsg, text);
    if (strcmp(headerMsg->GetDeviceName(), kMsgBody.RETRACT_PROBE.c_str()) == 0)
    {
      log.Log("Received Probe Retract Command From the Planning Software.",logger::INFO);
      // Send acknowledgment for retract needle command
      std::stringstream ss;
      ss << kCommunicationCmds.ACKNOWLEDGE << kMsgBody.RETRACT_PROBE;
      SendStatusMessage(ss.str().c_str(), igtl::StatusMessage::STATUS_OK, 0);
      // Robot is ready to retract to home upon pressing of the foot pedal.
      this->RStatus->RetractProbeToHome();
      // Send acknowledgment for successful needle retraction.
      SendStatusMessage(kMsgBody.RETRACT_PROBE.c_str(), igtl::StatusMessage::STATUS_OK, 0);
      return 1;
    }
    // TODO: Planning software can keep asking if probe is fully retracted and the reply can be sent from here.
  }
  return 0;
}

bool NeuroRobotMoveToTargetPhase::IsTransitionAllowed(const std::string &desired_next_workphase)
{
  // If it's an Emergency OR Stop transition allow it regardless of the needle condition.
  if ((desired_next_workphase == kStateNames.EMERGENCY) || (desired_next_workphase == kStateNames.STOP))
  {
    return true;
  }
  // Don't allow transition if the probe is being retracted OR not at home.
  if ((robot->clinical_mode->isRetractingProbe()) || (!robot->clinical_mode->isProbeAtHome()))
  {
    return false;
  }
  return NeuroRobotMoveToTargetPhase::IsTransitionAllowed(desired_next_workphase);
}