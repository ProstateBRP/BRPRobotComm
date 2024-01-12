#ifndef __ProstateRobotMoveToTargetPhase_HPP_
#define __ProstateRobotMoveToTargetPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotMoveToTargetPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotMoveToTargetPhase();
  ~ProstateRobotMoveToTargetPhase();

  virtual const char *Name() { return kStateNames.MOVE_TO_TARGET.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__ProstateRobotMoveToTargetPhase_HPP_
