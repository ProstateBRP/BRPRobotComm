include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Timer)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Logger)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)

add_library(CurvSteering
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/CurvSteering.cpp
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/SteeringAlgorithm.cpp
)
