#ifndef __ProstateRobotMotionController_HPP_
#define __ProstateRobotMotionController_HPP_

#include "ProstateRobotMotors.hpp"

class ProstateRobotMotionController
{
public:
    // Constructor
    ProstateRobotMotionController(ProstateRobotMotors *);
    // Functions
    FPGA_Utilities *fpga_utils;
    ProstateRobotMotors *motors;
    
    bool Spin();
    void UpdateMotorReadings();
    void UpdateMotorsStallDetect();
    void StopRobotMotion();
    void MoveAllMotors();
    void MoveMotor(ProstateRobotMotor);
    void StopMotor(ProstateRobotMotor);
    bool CheckForStalls();
    void DisableAllMotors();

};

#endif //__ProstateRobotMotionController_HPP_