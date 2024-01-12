#ifndef __ProstateRobotStartUpPhase_HPP_
#define __ProstateRobotStartUpPhase_HPP_

#include "ProstateRobotPhaseBase.hpp"

class ProstateRobotStartUpPhase : public ProstateRobotPhaseBase
{
public:
  ProstateRobotStartUpPhase();
  ~ProstateRobotStartUpPhase();

  virtual const char *Name() { return kStateNames.START_UP.c_str(); };
  virtual int Initialize();
  virtual int MessageHandler(igtl::MessageHeader *headerMsg);
  virtual void OnExit();
};

#endif //__ProstateRobotStartUpPhase_HPP_
