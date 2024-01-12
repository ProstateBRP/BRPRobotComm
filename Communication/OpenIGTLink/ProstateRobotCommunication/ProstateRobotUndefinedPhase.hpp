#ifndef __ProstateRobotUndefinedPhase_HPP_
#define __ProstateRobotUndefinedPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotPhaseBase;

class ProstateRobotUndefinedPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotUndefinedPhase();
  ~ProstateRobotUndefinedPhase();

  virtual const char *Name() { return kStateNames.UNDEFINED.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotUndefinedPhase_HPP_
