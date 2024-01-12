#ifndef __ProstateRobotEmergencyPhase_HPP_
#define __ProstateRobotEmergencyPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotEmergencyPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotEmergencyPhase();
  ~ProstateRobotEmergencyPhase();

  virtual const char *Name() { return kStateNames.EMERGENCY.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
  virtual bool IsTransitionAllowed(const std::string &);
};

#endif //__ProstateRobotEmergencyPhase_HPP_
