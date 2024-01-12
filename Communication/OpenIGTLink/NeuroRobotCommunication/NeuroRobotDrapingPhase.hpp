#ifndef __NeuroRobotDrapingPhase_HPP_
#define __NeuroRobotDrapingPhase_HPP_

#include "NeuroRobotPhaseBase.hpp"

class NeuroRobotDrapingPhase : public NeuroRobotPhaseBase
{
public:
  NeuroRobotDrapingPhase();
  ~NeuroRobotDrapingPhase();

  virtual const char *Name() { return kStateNames.DRAPING.c_str(); }
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__NeuroRobotDrapingPhase_HPP_
