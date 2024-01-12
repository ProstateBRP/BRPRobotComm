cmake_minimum_required(VERSION 3.1)

add_library(OpenIGTLinkComm
  OpenIGTLink.cpp
  ProstateRobotPhaseBase.cpp
  ProstateRobotCalibrationPhase.cpp
  ProstateRobotCommunicationBase.cpp
  ProstateRobotEmergencyPhase.cpp
  ProstateRobotManualPhase.cpp
  ProstateRobotMoveToTargetPhase.cpp
  ProstateRobotPlanningPhase.cpp
  ProstateRobotStartUpPhase.cpp
  ProstateRobotStatus.cpp
  ProstateRobotStopPhase.cpp
  ProstateRobotTargetingPhase.cpp
  ProstateRobotUndefinedPhase.cpp
)

TARGET_LINK_LIBRARIES(OpenIGTLinkComm OpenIGTLink)
