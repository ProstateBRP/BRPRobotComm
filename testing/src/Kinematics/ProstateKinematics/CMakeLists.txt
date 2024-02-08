include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics)

add_library(ProstateKinematics
  ${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/ProstateKinematics/ProstateKinematics.cpp
)