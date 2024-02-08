include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(BicycleKinematics
${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/BicycleKinematics/BicycleKinematics.cpp
)

target_include_directories(BicycleKinematics INTERFACE ProstateRobotUtils)

target_link_libraries(BicycleKinematics INTERFACE ProstateRobotUtils)