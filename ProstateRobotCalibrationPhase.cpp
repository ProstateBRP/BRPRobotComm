#include "ProstateRobotCalibrationPhase.hpp"

ProstateRobotCalibrationPhase::ProstateRobotCalibrationPhase() : ProstateRobotPhaseBase()
{
  forbidden_transition_list = {
      kStateNames.CALIBRATION,
      kStateNames.UNDEFINED,
      kStateNames.TARGETING,
      kStateNames.MOVE_TO_TARGET
  };
}

ProstateRobotCalibrationPhase::~ProstateRobotCalibrationPhase()
{
}

int ProstateRobotCalibrationPhase::Initialize()
{
  this->SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
  return 1;
}

void ProstateRobotCalibrationPhase::OnExit()
{
}

int ProstateRobotCalibrationPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  /// Check if Transform message for calibration has been received
  if (strcmp(headerMsg->GetDeviceType(), kMsgTypes.TRANSFORM.c_str()) == 0 &&
      strncmp(headerMsg->GetDeviceName(), kCommunicationCmds.CALIBRATION.c_str(), 4) == 0)
  {
    igtl::Matrix4x4 matrix;
    this->ReceiveTransform(headerMsg, matrix);

    // Acknowledgement
    std::string devName = headerMsg->GetDeviceName();
    std::stringstream ss;
    ss << kCommunicationCmds.ACKNOWLEDGE << devName.substr(4, std::string::npos);

    SendTransformMessage(ss.str().c_str(), matrix);

    if (ValidateMatrix(matrix))
    {
      if (this->RStatus)
      {
        // Set robot's registration matrix
        this->RStatus->SetCalibrationMatrix(matrix);
        // Reporting back the reachable target pose
        igtl::Matrix4x4 reachable_target;
        RStatus->GetReachableTargetPoseMatrix(reachable_target);
        // This will update the navigation regarding the reachable target
        SendTransformMessage(kMsgBody.REACHABLE_TARGET.c_str(), reachable_target);
        // Logging the OIGTL communication
        Logger &log = Logger::GetInstance();
        log.Log("OpenIGTLink Registration Received and Set in Code", devName.substr(4, std::string::npos), logger::INFO, true);
        log.Log(matrix, this->Name(), devName.substr(4, std::string::npos), logger::INFO, true);
        SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_OK, 0);
      }
      else
      {
        SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_CONFIG_ERROR, 0);
      }
    }
    else
    {
      std::cerr << "ERROR: Invalid calibration matrix." << std::endl;
      SendStatusMessage(this->Name(), igtl::StatusMessage::STATUS_CONFIG_ERROR, 0);
    }

    return 1;
  }

  return 0;
}
