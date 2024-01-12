include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)

add_library(CustomWebServer
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/Server/CustomWebServer.cpp
)