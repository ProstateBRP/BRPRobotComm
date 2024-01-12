#include "ProstateRobotStatus.hpp"

ProstateRobotStatus::ProstateRobotStatus(shared_ptr<ProstateRobot> prostate_robot) : robot(prostate_robot)
{
}

ProstateRobotStatus::~ProstateRobotStatus()
{
}

void ProstateRobotStatus::SetCalibrationMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> registration;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      registration(i, j) = matrix[i][j];
    }
  }
  robot->SetRegistration(registration);
}

int ProstateRobotStatus::GetCalibrationMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> calibration = robot->GetRegistration();
  if (this->robot->GetCalibrationFlag())
  {
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        matrix[i][j] = calibration(i, j);
      }
    }
    return 1;
  }
  else
  {
    return 0;
  }
}

void ProstateRobotStatus::SetTargetMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> target_image_coord;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      target_image_coord(i, j) = matrix[i][j];
    }
  }
  robot->SetTargetImageCoord(target_image_coord);
}

void ProstateRobotStatus::UpdateTargetMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> target_image_coord;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      target_image_coord(i, j) = matrix[i][j];
    }
  }
  robot->UpdateTargetImageCoord(target_image_coord);
}

/*!
Returns Target pose in imager coordinate system.
*/
int ProstateRobotStatus::GetTargetMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> target_image_coord = robot->GetTargetImageCoord();
  if (this->robot->GetTargetFlag())
  {
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        matrix[i][j] = target_image_coord(i, j);
      }
    }
    return 1;
  }
  else
  {
    return 0;
  }
}

/*!
Returns kinematically viable target pose in imager frame.
*/
void ProstateRobotStatus::GetReachableTargetPoseMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> reachable_target_image_coord = robot->GetReachableTargetImageCoord();
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      matrix[i][j] = reachable_target_image_coord(i, j);
    }
  }
}

/*!
Returns current tip pose of the robot in imager frame.
*/
void ProstateRobotStatus::GetCurrentPosition(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> current_pose_image_coord = robot->GetCurrentPositionImageCoord();
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      matrix[i][j] = current_pose_image_coord(i, j);
    }
  }
}

void ProstateRobotStatus::GetCurrentKinematicTipPosition(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> baseToTreatmentImageCoord = this->robot->GetCurrentPositionKinematicImageCoord();
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      matrix[i][j] = baseToTreatmentImageCoord(i, j);
    }
  }
}

void ProstateRobotStatus::SetNeedleLength(const double &needle_len)
{
  this->robot->SetNeedleLength(needle_len);
}

void ProstateRobotStatus::RetractNeedleToHome()
{
  robot->clinical_mode->PrepareNeedleRetract();
  // Clean up reported tip positions
  robot->clinical_mode->CleanUp();
}

void ProstateRobotStatus::SetCurvMethod(const string &ss)
{
  Logger &log = Logger::GetInstance();
  if (ss == string("UNIDIRECTIONAL"))
  {
    robot->clinical_mode->SetCurvMethod(CurvMethod::UNIDIRECTIONAL);
    robot->active_steering_mode->SetCurvMethod(CurvMethod::UNIDIRECTIONAL);  
    log.Log("Changed Curv Mode to UNIDIRECTIONAL", logger::INFO, true);
  }
  else if (ss == string("BIDIRECTIONAL"))
  {
    robot->clinical_mode->SetCurvMethod(CurvMethod::BIDIRECTIONAL);
    robot->active_steering_mode->SetCurvMethod(CurvMethod::BIDIRECTIONAL);
    log.Log("Changed Curv Mode to BIDIRECTIONAL", logger::INFO, true);
  }
  else
  {
    throw std::invalid_argument(ss + " is Undefined Mode!");
  }
}
void ProstateRobotStatus::SetAlpha(const double &alpha)
{
  try
  {
    robot->active_steering_mode->SetAlpha(alpha);
  }
  catch (...)
  {
    std::cerr << "Active Steering Mode Does not Exist!" << std::endl;
  }

  Logger &log = Logger::GetInstance();
  string log_msg = "Changed ALPHA value to: " + to_string(alpha) + ".";
  log.Log(log_msg, logger::INFO, true);
}

void ProstateRobotStatus::SetTargetAngle(const double &theta_d)
{
  try
  {
    // Convert to Radian
    double theta_d_rad = theta_d * M_PI / 180;
    robot->active_steering_mode->SetTargetAngle(theta_d_rad);
  }
  catch (...)
  {
    std::cerr << "Problem Setting the Target Angle!" << std::endl;
    return;
  }
  Logger &log = Logger::GetInstance();
  string log_msg = "Changed TARGET ANGLE value to: " + to_string(theta_d) + " degrees.";
  log.Log(log_msg, logger::INFO, true);
}

void ProstateRobotStatus::PushBackActualNeedlePos(const igtl::Matrix4x4 &matrix)
{
  // Convert to Eigen matrix
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> reported_needle_pos_image_coord;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      reported_needle_pos_image_coord(i, j) = matrix[i][j];
    }
  }

  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> reported_needle_tip_robot_coord = robot->ConvertFromImagerToRobotBase(reported_needle_pos_image_coord);
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> needle_tip_pos(reported_needle_tip_robot_coord(0, 3), reported_needle_tip_robot_coord(1, 3),
                                                               reported_needle_tip_robot_coord(2, 3));
  // If the pose estimation is successful the curv params will be updated.
  if (this->robot->clinical_mode->PushBackActualNeedlePosAndUpdatePose(needle_tip_pos))
  {
    this->robot->clinical_mode->UpdateCurvParamsAndInsertionLength();
  }
}

void ProstateRobotStatus::SetDeadband(const int val)
{
  robot->clinical_mode->SetDeadband(val);
}

void ProstateRobotStatus::SetSteeringMethod(const std::string &steering_type)
{
  if (steering_type == "CURV")
  {
    robot->clinical_mode->SetSteeringType(SteeringType::CURV);
    Logger &log = Logger::GetInstance();
    string log_msg = "Changed Steering type to: " + steering_type + ".";
    log.Log(log_msg, logger::INFO, true);
  }
  else if (steering_type == "FLIPPED")
  {
    robot->clinical_mode->SetSteeringType(SteeringType::FLIPPED);
    Logger &log = Logger::GetInstance();
    string log_msg = "Changed Steering type to: " + steering_type + ".";
    log.Log(log_msg, logger::INFO, true);
  }
  else
  {
    throw std::invalid_argument("Operation not successful!");
  }
}