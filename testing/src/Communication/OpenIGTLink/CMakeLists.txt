# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)

# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/NeuroRobotCommunication)
include_directories(/home/yiwei/brp/ProstateRobotCommunication)

add_library(OpenIGTLinkComm
  /home/yiwei/brp/ProstateRobotCommunication/testing/OpenIGTLink/OpenIGTLink.cpp

  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotPhaseBase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotStatus.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotCalibrationPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotCommunicationBase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotEmergencyPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotManualPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotMoveToTargetPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotPlanningPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotStartUpPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotStopPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotTargetingPhase.cpp
  /home/yiwei/brp/ProstateRobotCommunication/ProstateRobotUndefinedPhase.cpp

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
