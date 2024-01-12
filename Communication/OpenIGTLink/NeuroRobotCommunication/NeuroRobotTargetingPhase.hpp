#ifndef __NeuroRobotTargetingPhase_HPP_
#define __NeuroRobotTargetingPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotTargetingPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotTargetingPhase();
  ~NeuroRobotTargetingPhase();

  virtual const char *Name() { return kStateNames.TARGETING.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotTargetingPhase_HPP_
