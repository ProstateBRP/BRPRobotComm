#ifndef __NeuroRobotCalibrationPhase_HPP_
#define __NeuroRobotCalibrationPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotCalibrationPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotCalibrationPhase();
  ~NeuroRobotCalibrationPhase();

  virtual const char *Name() { return kStateNames.CALIBRATION.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotCalibrationPhase_HPP_
