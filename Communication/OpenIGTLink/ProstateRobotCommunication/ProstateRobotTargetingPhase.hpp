#ifndef __ProstateRobotTargetingPhase_HPP_
#define __ProstateRobotTargetingPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotTargetingPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotTargetingPhase();
  ~ProstateRobotTargetingPhase();

  virtual const char *Name() { return kStateNames.TARGETING.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotTargetingPhase_HPP_
