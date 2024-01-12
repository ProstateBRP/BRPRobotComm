#ifndef __ProstateRobotPlanningPhase_HPP_
#define __ProstateRobotPlanningPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotPlanningPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotPlanningPhase();
  ~ProstateRobotPlanningPhase();

  virtual const char *Name() { return kStateNames.PLANNING.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotPlanningPhase_HPP_
