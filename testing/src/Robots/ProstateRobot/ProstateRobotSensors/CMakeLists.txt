include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(ProstateRobotSensors
${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/ProstateRobotSensors.cpp
)

target_include_directories(ProstateRobotSensors INTERFACE ProstateRobotUtils)

target_link_libraries(ProstateRobotSensors INTERFACE ProstateRobotUtils)


