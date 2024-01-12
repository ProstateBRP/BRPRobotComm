#ifndef __NeuroRobotStatus_HPP_
#define __NeuroRobotStatus_HPP_

#include "igtlSocket.h"
#include "igtlMath.h"
#include "igtlMessageBase.h"

#include "NeuroRobot.hpp"
#include "NeuroRobotCommunicationBase.hpp"

class NeuroRobotStatus
{
public:
  NeuroRobotStatus(shared_ptr<NeuroRobot>);
  ~NeuroRobotStatus();
  bool GetTargetFlag() { return this->robot->GetTargetFlag(); }
  bool GetCalibrationFlag() { return this->robot->GetCalibrationFlag(); };
  int GetEntryFlag() { return this->robot->GetEntryFlag(); }
  int GetCalibrationMatrix(igtl::Matrix4x4 &matrix);
  void SetCalibrationMatrix(igtl::Matrix4x4 &matrix);
  int GetTargetMatrix(igtl::Matrix4x4 &matrix);
  void SetTargetMatrix(igtl::Matrix4x4 &matrix);
  int GetEntryMatrix(igtl::Matrix4x4 &matrix);
  void SetEntryMatrix(igtl::Matrix4x4 &matrix);
  void GetCurrentPosition(igtl::Matrix4x4 &currentPosition);
  void GetReachableTargetPoseMatrix(igtl::Matrix4x4 &matrix);
  void RetractProbeToHome();
  void SetProbeSpecs(const Probe &);

  shared_ptr<NeuroRobot> robot{nullptr};
  NeuroRobotStates kStateNames;
};

#endif //__NeuroRobotStatus_HPP_
