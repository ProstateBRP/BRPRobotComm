#ifndef __ProstateRobotPhaseBase_HPP_
#define __ProstateRobotPhaseBase_HPP_

#include "ProstateRobotCommunicationBase.hpp"

#include <cmath>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "igtlSocket.h"
#include "igtlMath.h"
#include "igtlMessageBase.h"
#include "igtlOSUtil.h"
#include "igtlStringMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"
#include "igtlTransformMessage.h"
#include "ProstateRobotStatus.hpp"

class ProstateRobotPhaseBase : public ProstateRobotCommunicationBase
{
public:
  ProstateRobotPhaseBase();
  virtual ~ProstateRobotPhaseBase(); // Changed the destructor to virtual
  shared_ptr<ProstateRobot> robot{nullptr};
  virtual const char *Name() = 0;

  // Enter() will be called when the workphase is switched from another
  // workphase. Enter() calls Initialize() which implements actual
  // initialization process for this workphase.
  virtual int Enter(const char *queryID);

  // Initialization process. This must be implemented in child classes.
  virtual int Initialize() = 0;

  // Process() will be called by the main session loop.
  // It checks if any work phase change request is received first. If not it calls
  // MessageHandler() to perform workphase-specific message handling.
  // Returns 1 if there is any workphase change request. Otherwise returns 0.
  virtual int Process();

  /*
  This method is use to clean up the current state upon changing to a new one. Specific behavior can be defined in the
  inherited classes.
  */
  virtual void OnExit();

  // MessageHandler() defines workphase-specific message handling.
  // The function needs to be implemented in child classes.
  virtual int MessageHandler(igtl::MessageHeader *headerMsg); // Message handler
  // Checks if the transition to the requested workphase from the current workphase is allowed. Returns true if it is allowed.
  virtual bool IsTransitionAllowed(const std::string &);

  void SetPreviousWorkPhase(std::string prev_phase) { this->PreviousWorkphase = prev_phase; }
  std::string GetPreviousWorkPhase() { return this->PreviousWorkphase; }
  std::string GetNextWorkPhase() { return this->NextWorkphase; };
  std::string GetQueryID() { return this->QueryID; };

  void SetRobot(shared_ptr<ProstateRobot> prostate_robot) { this->robot = prostate_robot; }
  void SetRobotStatus(shared_ptr<ProstateRobotStatus> rs) { this->RStatus = rs; };
  // Get robot status
  shared_ptr<ProstateRobotStatus> GetRobotStatus() { return this->RStatus; };

protected:
  // Check if a CMD message (workphase change) has been received.
  // Return 1, if workphase change has been requested.
  int CheckWorkphaseChange(igtl::MessageHeader *headerMsg);

  // Check if there is any messages that must be accepted
  // regardless of current workhpase.
  int CheckCommonMessage(igtl::MessageHeader *headerMsg);

  std::string PreviousWorkphase;
  std::string NextWorkphase;
  std::string QueryID;
  std::vector<std::string> forbidden_transition_list{};
  ProstateRobotStates kStateNames;
  ProstateRobotMessageTypes kMsgTypes;
  ProstateRobotCommunicationCommands kCommunicationCmds;
  ProstateRobotMessageBody kMsgBody;
  shared_ptr<ProstateRobotStatus> RStatus{nullptr};
};

#endif //__ProstateRobotPhaseBase_HPP_
