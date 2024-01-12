#include "NeuroRobotManualPhase.hpp"

NeuroRobotManualPhase::NeuroRobotManualPhase() : NeuroRobotPhaseBase()
{
  forbidden_transition_list = {kStateNames.MANUAL, kStateNames.UNDEFINED};
}

NeuroRobotManualPhase::~NeuroRobotManualPhase()
{
}

void NeuroRobotManualPhase::OnExit()
{
}

int NeuroRobotManualPhase::Initialize()
{

  // Send Status after waiting for 2 seconds (mimicking initialization process)
  this->SendStatusMessage(this->Name(), 1, 0);
  return 1;
}

// What does this function do exactly? It seems that it is always returning zero.
int NeuroRobotManualPhase::MessageHandler(igtl::MessageHeader *headerMsg)
{
  return 0;
}
