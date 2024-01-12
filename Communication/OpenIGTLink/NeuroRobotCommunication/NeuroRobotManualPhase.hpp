#ifndef __NeuroRobotManualPhase_HPP_
#define __NeuroRobotManualPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotManualPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotManualPhase();
  ~NeuroRobotManualPhase();

  virtual const char *Name() { return kStateNames.MANUAL.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotManualPhase_HPP_
