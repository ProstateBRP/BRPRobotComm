#ifndef __ProstateRobotManualPhase_HPP_
#define __ProstateRobotManualPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotManualPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotManualPhase();
  ~ProstateRobotManualPhase();

  virtual const char *Name() { return kStateNames.MANUAL.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotManualPhase_HPP_
