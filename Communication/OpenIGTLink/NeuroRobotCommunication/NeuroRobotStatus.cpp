#include "NeuroRobotStatus.hpp"

NeuroRobotStatus::NeuroRobotStatus(shared_ptr<NeuroRobot> neuro_robot_ptr) : robot(neuro_robot_ptr)
{
}

NeuroRobotStatus::~NeuroRobotStatus()
{
}

void NeuroRobotStatus::SetEntryMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> entry_image_coord;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      entry_image_coord(i, j) = matrix[i][j];
    }
  }
  robot->SetEntryImageCoord(entry_image_coord);
}

void NeuroRobotStatus::SetTargetMatrix(igtl::Matrix4x4 &matrix)
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

void NeuroRobotStatus::SetCalibrationMatrix(igtl::Matrix4x4 &matrix)
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

/*!
Returns Target pose in imager coordinate system.
*/
int NeuroRobotStatus::GetEntryMatrix(igtl::Matrix4x4 &matrix)
{
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> entry_image_coord = robot->GetEntryImageCoord();
  if (this->robot->GetEntryFlag())
  {
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        matrix[i][j] = entry_image_coord(i, j);
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
Returns Target pose in imager coordinate system.
*/
int NeuroRobotStatus::GetTargetMatrix(igtl::Matrix4x4 &matrix)
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

int NeuroRobotStatus::GetCalibrationMatrix(igtl::Matrix4x4 &matrix)
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

void NeuroRobotStatus::SetProbeSpecs(const Probe &probe)
{
  this->robot->SetProbeSpecs(probe);
}

void NeuroRobotStatus::RetractProbeToHome()
{
  robot->clinical_mode->PrepareProbeRetract();
}

/*!
Returns kinematically viable target pose in imager frame.
*/
void NeuroRobotStatus::GetReachableTargetPoseMatrix(igtl::Matrix4x4 &matrix)
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
void NeuroRobotStatus::GetCurrentPosition(igtl::Matrix4x4 &matrix)
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