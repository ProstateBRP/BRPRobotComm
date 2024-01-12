#ifndef __NeuroRobotStopPhase_HPP_
#define __NeuroRobotStopPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotStopPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotStopPhase();
  ~NeuroRobotStopPhase();

  virtual const char *Name() { return kStateNames.STOP.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__NeuroRobotStopPhase_HPP_
