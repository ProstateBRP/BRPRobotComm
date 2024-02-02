include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(ProstateRobotMotors
${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/ProstateRobotMotors.cpp
)

target_include_directories(ProstateRobotMotors INTERFACE ProstateRobotUtils)

target_link_libraries(ProstateRobotMotors INTERFACE ProstateRobotUtils)


