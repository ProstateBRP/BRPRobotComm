#ifndef __NeuroRobotPlanningPhase_HPP_
#define __NeuroRobotPlanningPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotPlanningPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotPlanningPhase();
  ~NeuroRobotPlanningPhase();

  virtual const char *Name() { return kStateNames.PLANNING.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotPlanningPhase_HPP_
