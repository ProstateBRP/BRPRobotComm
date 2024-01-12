#ifndef __ProstateRobotStatus_HPP_
#define __ProstateRobotStatus_HPP_

#include "igtlSocket.h"
#include "igtlMath.h"
#include "igtlMessageBase.h"

#include "ProstateRobot.hpp"
#include "ProstateRobotCommunicationBase.hpp"

class ProstateRobotStatus
{
public:
  ProstateRobotStatus(shared_ptr<ProstateRobot>);
  ~ProstateRobotStatus();
  bool GetTargetFlag() { return this->robot->GetTargetFlag(); }
  bool GetCalibrationFlag() { return this->robot->GetCalibrationFlag(); };
  int GetCalibrationMatrix(igtl::Matrix4x4 &);
  void SetCalibrationMatrix(igtl::Matrix4x4 &);
  int GetTargetMatrix(igtl::Matrix4x4 &);
  void SetTargetMatrix(igtl::Matrix4x4 &);
  void GetCurrentPosition(igtl::Matrix4x4 &);
  void GetCurrentKinematicTipPosition(igtl::Matrix4x4 &);
  void GetReachableTargetPoseMatrix(igtl::Matrix4x4 &);
  void RetractNeedleToHome();
  void SetCurvMethod(const string &);
  void SetNeedleLength(const double &);
  void PushBackActualNeedlePos(const igtl::Matrix4x4 &);
  void SetAlpha(const double &);
  void UpdateTargetMatrix(igtl::Matrix4x4 &);
  void SetTargetAngle(const double &);
  void SetDeadband(const int);
  void SetSteeringMethod(const std::string &);
  // Atributes
  shared_ptr<ProstateRobot> robot{nullptr};
  ProstateRobotStates kStateNames;
};

#endif //__ProstateRobotStatus_HPP_

