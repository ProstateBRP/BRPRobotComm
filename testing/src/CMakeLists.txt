cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
PROJECT(COMM_MODULE_Test)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# call a CMake module, search the file system for the appropriate threads package for this platform
# set the CMAKE_THREAD_LIBS_INIT variable
find_package(Threads REQUIRED)

set(EIGEN_DIR /home/yiwei/libs/eigen-3.4.0)
set(IGTL_DIR /usr/local/include/igtl)
set(BOOST_DIR /home/yiwei/libs/boost_1_80_0)
set(SURGICAL_ROBOT_SOURCE_DIR /home/yiwei/brp/ProstateRobotCommunication/testing)
set(PROSTATE_COMM_SOURCE_DIR /home/yiwei/brp/ProstateRobotCommunication)

include_directories(${EIGEN_DIR} ${BOOST_DIR} ${IGTL_DIR})


# header search path
# include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/NeuroRobot/NeuroRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotUtils/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotSensors/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotKinematicsController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotMotionController/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/ProstateRobotModes/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/)
include_directories(${PROSTATE_COMM_SOURCE_DIR})
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Logger/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Timer/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/PolynomialFit/)
include_directories(${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/ProstateKinematics)
set(COMM_MODULE_TEST_SOURCE CommModuleTest.cpp)
ADD_EXECUTABLE(CommModuleTest ${COMM_MODULE_TEST_SOURCE})

add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/NeedleSteeringAlgorithms/CurvSteering/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Robots/ProstateRobot/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Communication/OpenIGTLink/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Logger/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/Timer/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Utilities/PolynomialFit/)
add_subdirectory(${SURGICAL_ROBOT_SOURCE_DIR}/src/Kinematics/ProstateKinematics)

TARGET_LINK_LIBRARIES(CommModuleTest 

  Threads::Threads
  OpenIGTLinkComm
  Logger
  Timer
  ProstateKinematics
  ProstateRobot
  CurvSteering
  ProstateRobotModeBase
  ProstateRobotClinicalMode
  ProstateRobotAutomatedHomingMode
  ProstateRobotManualMode
  ProstateRobotFrequencySweepMode
  ProstateRobotActiveSteeringMode
  )