include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)

add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)

add_library(ProstateRobot
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobot.cpp)

target_link_libraries(ProstateRobot
  ProstateRobotUtils
  ProstateKinematics
  BicycleKinematics
  CurvSteering
  PolyFit
  ForceSensor
  ProstateRobotKinematicsController
  ProstateRobotMotionController
  ProstateRobotSensors
  ProstateRobotMotors
  ProstateRobotModeBase
  ProstateRobotClinicalMode
  ProstateRobotAutomatedHomingMode
  ProstateRobotManualMode
  ProstateRobotFrequencySweepMode
  ProstateRobotActiveSteeringMode
)
