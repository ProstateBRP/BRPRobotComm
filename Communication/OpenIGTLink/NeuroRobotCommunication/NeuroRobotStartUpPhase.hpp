#ifndef __NeuroRobotStartUpPhase_HPP_
#define __NeuroRobotStartUpPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotStartUpPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotStartUpPhase();
  ~NeuroRobotStartUpPhase();

  virtual const char *Name() { return kStateNames.START_UP.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__NeuroRobotStartUpPhase_HPP_
