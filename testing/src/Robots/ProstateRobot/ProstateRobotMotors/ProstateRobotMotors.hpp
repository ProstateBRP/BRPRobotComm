#ifndef _ProstateRobotMotors_HPP_
#define _ProstateRobotMotors_HPP_

#include "Timer.hpp"
#include "Motor.hpp"
#include "Logger.hpp"
#include "Packets.hpp"
#include "FPGA_Utilities.hpp"
#include "ProstateRobotEnumerations.hpp"
#include "ProstateRobotConstants.hpp"

class ProstateRobotMotors
{
public:
    ProstateRobotMotors(Packets *, FPGA_Utilities *);
    std::vector<Motor *> GetMotorsListVector();
    Motor *GetMotor(ProstateRobotMotor);
    void SetDesiredSetpoint(ProstateRobotMotor, int);
    int CalculateMotionDiff(ProstateRobotMotor, int);
    void SaveSetpointList(ProstateRobotMotor, int, int);
    void SetCurrentEncoderReadingAsHomeForAllMotors();
    bool IsMotorAtLimit(ProstateRobotMotor);
    bool CheckForStalls();
    ProstateRobotMotorMapPtr GetMotorsListMap(){return motor_list;}

    ProstateRobotMotorNames MOTOR;
    ProstateRobotUnitNames UNIT;
    // Outgoing and Incoming Packets via SPI
    Packets *_packets;
    // The FPGA Utility object was included to allow for control over LEDs
    FPGA_Utilities *_fpga_util;

private:
    // Prostate robot motors are defined below
    Motor_Config FrontLeftConfig, FrontRightConfig, BackLeftConfig, BackRightConfig, InsertionConfig, RotationConfig;
    Motor front_left, front_right, back_left, back_right, insertion, rotation;
    ProstateRobotMotorMapPtr motor_list;
};

#endif // _ProstateRobotMotors_HPP_