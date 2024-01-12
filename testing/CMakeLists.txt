PROJECT(BRPRobotIGTLTest)

cmake_minimum_required(VERSION 3.1)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(EIGEN_DIR /home/yiwei/libs/eigen-3.4.0)
set(IGTL_DIR /usr/local/include/igtl)
set(BOOST_DIR /home/yiwei/libs/boost_1_80_0)

include_directories(${EIGEN_DIR} ${BOOST_DIR} ${IGTL_DIR})

# find_package(OpenIGTLink REQUIRED)
# include(${OpenIGTLink_USE_FILE})

include_directories(./Robots/)

add_subdirectory(/home/yiwei/brp/ProstateRobotCommunication/testing/Robots/ProstateRobot)
include_directories(/home/yiwei/brp/ProstateRobotCommunication)

set(ROBOT_SIM_SOURCE
  main.cxx
)

ADD_EXECUTABLE(RobotTest ${ROBOT_SIM_SOURCE})
TARGET_LINK_LIBRARIES(RobotTest 
Threads::Threads
OpenIGTLinkComm
ProstateRobot
)