#ifndef __NeuroRobotPhaseBase_HPP_
#define __NeuroRobotPhaseBase_HPP_

#include "NeuroRobotCommunicationBase.hpp"

#include <cmath>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "igtlMath.h"
#include "igtlSocket.h"
#include "igtlOSUtil.h"
#include "igtlStringMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"
#include "igtlTransformMessage.h"
#include "igtlMessageBase.h"
#include "NeuroRobotStatus.hpp"

class NeuroRobotPhaseBase : public NeuroRobotCommunicationBase
{
public:
  NeuroRobotPhaseBase();
  virtual ~NeuroRobotPhaseBase();        // Changed the destructor to virtual
  shared_ptr<NeuroRobot> robot{nullptr}; // Robot object
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

  void SetRobot(shared_ptr<NeuroRobot> neuro_robot) { this->robot = neuro_robot; }
  void SetRobotStatus(shared_ptr<NeuroRobotStatus> rs) { this->RStatus = rs; };
  // Get robot status
  shared_ptr<NeuroRobotStatus> GetRobotStatus() { return this->RStatus; };

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
  NeuroRobotStates kStateNames;
  NeuroRobotMessageTypes kMsgTypes;
  NeuroRobotCommunicationCommands kCommunicationCmds;
  NeuroRobotMessageBody kMsgBody;
  shared_ptr<NeuroRobotStatus> RStatus{nullptr};
};

#endif //__NeuroRobotPhaseBase_HPP_
