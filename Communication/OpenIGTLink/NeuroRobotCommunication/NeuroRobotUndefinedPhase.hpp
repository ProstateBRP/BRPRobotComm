#ifndef __NeuroRobotUndefinedPhase_HPP_
#define __NeuroRobotUndefinedPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotUndefinedPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotUndefinedPhase();
  ~NeuroRobotUndefinedPhase();

  virtual const char *Name() { return kStateNames.UNDEFINED.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotUndefinedPhase_HPP_
