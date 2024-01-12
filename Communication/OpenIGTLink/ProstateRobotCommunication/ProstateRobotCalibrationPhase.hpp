#ifndef __ProstateRobotCalibrationPhase_HPP_
#define __ProstateRobotCalibrationPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotCalibrationPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotCalibrationPhase();
  ~ProstateRobotCalibrationPhase();

  virtual const char *Name() { return kStateNames.CALIBRATION.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotCalibrationPhase_HPP_
