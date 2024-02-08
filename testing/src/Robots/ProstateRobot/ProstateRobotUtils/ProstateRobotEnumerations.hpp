#ifndef _ProstateRobotEnumeration_HPP_
#define _ProstateRobotEnumeration_HPP_

#include <string>
#include <map>

// #include "Motor.hpp"

enum class ProstateRobotMotor : int
{
    ROTATION = 0,
    INSERTION = 1,
    BACK_LEFT = 2,
    FRONT_LEFT = 4,
    BACK_RIGHT = 5,
    FRONT_RIGHT = 6,
};

enum class ProstateRobotAnalogDigitalId : int
{
    INPUT_SENSOR = 2,
    NEEDLE_SENSOR = 3,

};

enum class ProstateRobotForceSensor : int
{
    FORCE_SENSOR = 3
};

struct ProstateRobotMotorNames
{

    std::string BACK_LEFT{"Back Left"};
    std::string BACK_RIGHT{"Back Right"};
    std::string FRONT_LEFT{"Front Left"};
    std::string FRONT_RIGHT{"Front Right"};
    std::string INSERTION{"Insertion"};
    std::string ROTATION{"Rotation"};
};

struct ProstateRobotUnitNames
{
    std::string MM{"mm"};
    std::string RADIAN{"rad"};
    std::string DEGREE{"degree"};
};

struct ProstateRobotModes
{
    std::string CLINICAL_MODE{"Clinical Mode"};
    std::string MANUAL_MODE{"Manual Mode"};
    std::string AUTOMATED_HOMING_MODE{"Automated Homing Mode"};
    std::string FREQUENCY_SWEEP_MODE{"Frequency Sweep Mode"};
    std::string ACTIVE_STEERING_MODE{"Active Steering Mode"};
};

struct ProstateRobotStates
{
    std::string UNDEFINED{"UNDEFINED"};
    std::string START_UP{"START_UP"};
    std::string CALIBRATION{"CALIBRATION"};
    std::string PLANNING{"PLANNING"};
    std::string TARGETING{"TARGETING"};
    std::string MOVE_TO_TARGET{"MOVE_TO_TARGET"};
    std::string STOP{"STOP"};
    std::string EMERGENCY{"EMERGENCY"};
    std::string MANUAL{"MANUAL"};
};

struct ProstateRobotCommunicationCommands
{
    const string CALIBRATION{"CLB_"};
    const string TARGET{"TGT_"};
    const string ACKNOWLEDGE{"ACK_"};
    const string COMMAND{"CMD_"};
    const string NEEDLE_POS{"NPOS_"};
};

struct ProstateRobotMessageTypes
{
    const string TRANSFORM{"TRANSFORM"};
    const string STATUS{"STATUS"};
    const string STRING{"STRING"};
};

struct ProstateRobotMessageBody
{
    const string CURRENT_STATUS{"CURRENT_STATUS"};
    const string CURV_METHOD{"CURV_METHOD"};
    const string ALPHA{"ALPHA"};
    const string CURRENT_POSITION{"CURRENT_POSITION"};
    const string IS_IN_TARGETING_POS{"IS_IN_TARGETING_POS"};
    const string HAS_REACHED_TARGET{"HAS_REACHED_TARGET"};
    const string RETRACT_NEEDLE{"RETRACT_NEEDLE"};
    const string NEEDLE_LENGTH{"NEEDLE_LENGTH"};
    const string TARGET{"TARGET"};
    const string REACHABLE_TARGET{"REACHABLE_TARGET"};
};

// Typedefs
typedef std::map<ProstateRobotMotor, int> ProstateRobotMotorSetpointMap;
// typedef std::map<ProstateRobotMotor, Motor *> ProstateRobotMotorMapPtr;

#endif //_ProstateRobotEnumeration_HPP_