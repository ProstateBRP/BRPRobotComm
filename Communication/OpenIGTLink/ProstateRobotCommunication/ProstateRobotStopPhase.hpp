#ifndef __ProstateRobotStopPhase_HPP_
#define __ProstateRobotStopPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotStopPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotStopPhase();
  ~ProstateRobotStopPhase();

  virtual const char *Name() { return kStateNames.STOP.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__ProstateRobotStopPhase_HPP_
