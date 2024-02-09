cmake_minimum_required(VERSION 3.13)

# set(OpenIGTLink_DIR /home/yiwei/libs/OpenIGTLink-build)

find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})

include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)

# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication)
include_directories(${PROSTATE_COMM_SOURCE_DIR})

add_library(OpenIGTLinkComm
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/OpenIGTLink.cpp

  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotPhaseBase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotStatus.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotCalibrationPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotCommunicationBase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotEmergencyPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotManualPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotMoveToTargetPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotPlanningPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotStartUpPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotStopPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotTargetingPhase.cpp
  ${PROSTATE_COMM_SOURCE_DIR}/ProstateRobotUndefinedPhase.cpp

  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotPhaseBase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStatus.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotCalibrationPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotCommunicationBase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotEmergencyPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotManualPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotMoveToTargetPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotPlanningPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStartUpPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStopPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotTargetingPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotUndefinedPhase.cpp
  # ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotDrapingPhase.cpp
)

TARGET_LINK_LIBRARIES(OpenIGTLinkComm OpenIGTLink ProstateRobot)
