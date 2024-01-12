#ifndef __NeuroRobotMoveToTargetPhase_HPP_
#define __NeuroRobotMoveToTargetPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotMoveToTargetPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotMoveToTargetPhase();
  ~NeuroRobotMoveToTargetPhase();

  virtual const char *Name() { return kStateNames.MOVE_TO_TARGET.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__NeuroRobotMoveToTargetPhase_HPP_
