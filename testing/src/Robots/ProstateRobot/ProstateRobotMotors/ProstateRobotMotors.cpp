#include "ProstateRobotMotors.hpp"
using namespace motorconfig;

ProstateRobotMotors::ProstateRobotMotors(Packets *packets, FPGA_Utilities *fpga_util) : _packets(packets), _fpga_util(fpga_util)
{
    //=================== Prostate Robot Motors ======================
    /* Motor Configurations
     Motor = { card_type, motor_type, name, encoderResolution, unit, unitConstant, motorDirection, directionCorrection, limitType, maxVelocity, minVelocity, minTicks, maxTicks, homeOffsetInTicks, Kp, Ki, Kd, deadband, lower_bound;
	 upper_bound};
    */
    FrontLeftConfig = {card_type::externaldriver_pwm, motor_type::shinsei, MOTOR.FRONT_LEFT, 5000, UNIT.MM, 2500, CLOCKWISE, CORRECTION, limit_type::upper, 0x0001ffff, 0x0, 100000, 472225, 375000, 100, 0, 0, 20, 0, 0};
    FrontRightConfig = {card_type::externaldriver_pwm, motor_type::shinsei, MOTOR.FRONT_RIGHT, 5000, UNIT.MM, 2500, COUNTER_CLOCKWISE, NO_CORRECTION, limit_type::lower, 0x0001ffff, 0x0, -472225, -100000, -375000, 100, 0, 0, 20, 0, 0};
    BackLeftConfig = {card_type::externaldriver_pwm, motor_type::shinsei, MOTOR.BACK_LEFT, 5000, UNIT.MM, 2500, CLOCKWISE, CORRECTION, limit_type::unspecified, 0x0001ffff, 0x0, 100000, 468475, 375000, 100, 0, 0, 20, 0, 0};
    BackRightConfig = {card_type::externaldriver_pwm, motor_type::shinsei, MOTOR.BACK_RIGHT, 5000, UNIT.MM, 2500, COUNTER_CLOCKWISE, NO_CORRECTION, limit_type::lower, 0x0001ffff, 0x0, -463475, -100000, -375000, 100, 0, 0, 20, 0, 0};
    InsertionConfig = {card_type::externaldriver_pwm, motor_type::shinsei, MOTOR.INSERTION, 5000, UNIT.MM, 1666.67, CLOCKWISE, NO_CORRECTION, limit_type::lower, 0x0003ffff, 0x0, 0, 280000, 0, 100, 0, 0, 20, 0, 0};
    RotationConfig = {card_type::highfrequency, motor_type::shinsei, MOTOR.ROTATION, 5000, UNIT.DEGREE, 265.27, COUNTER_CLOCKWISE, CORRECTION, limit_type::unspecified, 0x42B8DBB4, 0x0, -8073, 8073, 0, 2.0, 0.9, 0.15, 2, 0, 240};

    // These are the motor definitions
    front_left = Motor(packets, static_cast<int>(ProstateRobotMotor::FRONT_LEFT), FrontLeftConfig);
    front_right = Motor(packets, static_cast<int>(ProstateRobotMotor::FRONT_RIGHT), FrontRightConfig);
    back_left = Motor(packets, static_cast<int>(ProstateRobotMotor::BACK_LEFT), BackLeftConfig);
    back_right = Motor(packets, static_cast<int>(ProstateRobotMotor::BACK_RIGHT), BackRightConfig);
    insertion = Motor(packets, static_cast<int>(ProstateRobotMotor::INSERTION), InsertionConfig);
    rotation = Motor(packets, static_cast<int>(ProstateRobotMotor::ROTATION), RotationConfig);
    rotation._velocity = 0x45A0DB84;
    // Initialize motor list
    motor_list =
        {
            {ProstateRobotMotor::FRONT_LEFT, &front_left},
            {ProstateRobotMotor::FRONT_RIGHT, &front_right},
            {ProstateRobotMotor::BACK_LEFT, &back_left},
            {ProstateRobotMotor::BACK_RIGHT, &back_right},
            {ProstateRobotMotor::INSERTION, &insertion},
            {ProstateRobotMotor::ROTATION, &rotation},
        };
}

Motor *ProstateRobotMotors::GetMotor(ProstateRobotMotor cardID)
{

    if (motor_list.find(cardID) != motor_list.end())
    {
        // if the element is found before the end of the map
        return motor_list[cardID];
    }
    else
    {
        return NULL;
    }
}

void ProstateRobotMotors::SetDesiredSetpoint(ProstateRobotMotor motor_name, int setpoint)
{
    Motor *motor = motor_list[motor_name];
    motor->_setpoint = setpoint;
}

int ProstateRobotMotors::CalculateMotionDiff(ProstateRobotMotor motor_name, int setpoint)
{
    Motor *motor = motor_list[motor_name];
    int current = motor->GetEncoderPositionTicks();
    int motion_diff = abs(setpoint - current);

    return motion_diff;
}    

void ProstateRobotMotors::SaveSetpointList(ProstateRobotMotor motor_name, int setpoint, int step_num)
{
        Motor *motor = motor_list[motor_name];
    int current = motor->GetEncoderPositionTicks();
    
    //  clear setpoint list
    while (!motor->_setpointList.empty())
    {
        motor->_setpointList.pop();
    }

    //  create new setpoint list
    int step_size = (int)((setpoint - current) / step_num);
    if (current <= setpoint)
    {
        while (current + step_size < setpoint)
        {
            motor->_setpointList.push(current + step_size);
            current += step_size;
        }
    }
    else
    {
        while (current + step_size > setpoint)
        {
            motor->_setpointList.push(current + step_size);
            current += step_size;
        }
    }
    motor->_setpointList.push(setpoint);

    motor->_setpoint = motor->_setpointList.front();

    //  log length of setpoint list
    Logger &log = Logger::GetInstance();
    log.Log("Setpoint List Length: " + to_string(motor->_setpointList.size()), logger::INFO, true);
    log.Log("Setpoint List: ", logger::INFO, true);
    queue<int> temp = motor->_setpointList;
    while (!temp.empty())
    {
        log.Log(to_string(temp.front()), logger::INFO, true);
        temp.pop();
    }
        
}

std::vector<Motor *> ProstateRobotMotors::GetMotorsListVector()
{
    std::vector<Motor *> motor_vector;
    for (auto it = motor_list.begin(); it != motor_list.end(); it++)
    {
        motor_vector.push_back(it->second);
    }
    return motor_vector;
}

void ProstateRobotMotors::SetCurrentEncoderReadingAsHomeForAllMotors()
{
    for (auto it = motor_list.begin(); it != motor_list.end(); it++)
    {
        it->second->SetEncoderReference();
    }
}

bool ProstateRobotMotors::IsMotorAtLimit(ProstateRobotMotor motor_name)
{
    return motor_list[motor_name]->IsLimit();
}

bool ProstateRobotMotors::CheckForStalls()
{
    for (auto it = motor_list.begin(); it != motor_list.end(); it++)
    {
        if (it->second->_log_stall_failure_flag > 0)
        {
            return true;
        }
    }
    return false;
}