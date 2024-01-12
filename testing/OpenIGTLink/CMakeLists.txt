include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)

include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication)

add_library(OpenIGTLinkComm
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/OpenIGTLink.cpp

  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotPhaseBase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotStatus.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotCalibrationPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotCommunicationBase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotEmergencyPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotManualPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotMoveToTargetPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotPlanningPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotStartUpPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotStopPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotTargetingPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/ProstateRobotCommunication/ProstateRobotUndefinedPhase.cpp

  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotPhaseBase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStatus.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotCalibrationPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotCommunicationBase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotEmergencyPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotManualPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotMoveToTargetPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotPlanningPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStartUpPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotStopPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotTargetingPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotUndefinedPhase.cpp
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication/NeuroRobotDrapingPhase.cpp
)

TARGET_LINK_LIBRARIES(OpenIGTLinkComm OpenIGTLink ProstateRobot NeuroRobot)
