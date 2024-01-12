#ifndef __NeuroRobotEmergencyPhase_HPP_
#define __NeuroRobotEmergencyPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotEmergencyPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotEmergencyPhase();
  ~NeuroRobotEmergencyPhase();

  virtual const char *Name() { return kStateNames.EMERGENCY.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotEmergencyPhase_HPP_
