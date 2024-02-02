include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(ProstateRobotMotionController
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/ProstateRobotMotionController.cpp)

target_link_libraries(ProstateRobotMotionController
    ProstateRobotMotors
    ProstateRobotUtils
)