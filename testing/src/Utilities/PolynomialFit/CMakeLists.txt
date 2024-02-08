include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)

add_library(PolyFit
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/PolynomialFit/PolyFit.cxx)