include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/ProstateKinematics/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(ProstateRobotKinematicsController
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/ProstateRobotKinematicsController.cpp)

target_link_libraries(ProstateRobotKinematicsController
    ProstateRobotMotors
    ProstateKinematics
    ProstateRobotUtils)