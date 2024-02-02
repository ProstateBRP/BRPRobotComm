# Add the base class and its source file
add_library(ProstateRobotModeBase
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotModeBase.cpp
)

# Add the Clinical Mode class and its source file
add_library(ProstateRobotClinicalMode
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotClinicalMode.cpp
)

# Add the Automated Homing Mode class and its source file
add_library(ProstateRobotAutomatedHomingMode
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotAutomatedHomingMode.cpp
)

# Add the Manual Mode class and its source file
add_library(ProstateRobotManualMode
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotManualMode.cpp
)

# Add the Frequency Sweep Mode class and its source file
add_library(ProstateRobotFrequencySweepMode
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotFrequencySweepMode.cpp
)

# Add the Active Steering Mode class and its source file
add_library(ProstateRobotActiveSteeringMode
    ${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/ProstateRobotActiveSteeringMode.cpp
)

# May not be necessary
target_link_libraries(ProstateRobotModeBase
    ProstateRobotMotionController)

target_link_libraries(ProstateRobotClinicalMode
    ProstateRobotModeBase
    ProstateRobotMotionController
    CurvSteering)

target_link_libraries(ProstateRobotAutomatedHomingMode
    ProstateRobotModeBase
    ProstateRobotMotionController)

target_link_libraries(ProstateRobotManualMode
    ProstateRobotModeBase
    ProstateRobotMotionController)

target_link_libraries(ProstateRobotFrequencySweepMode
    ProstateRobotModeBase
    ProstateRobotMotionController)

target_link_libraries(ProstateRobotActiveSteeringMode
    ProstateRobotModeBase
    ProstateRobotMotionController
    CurvSteering)