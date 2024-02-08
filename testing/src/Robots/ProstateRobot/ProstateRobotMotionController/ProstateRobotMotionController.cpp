#include "ProstateRobotMotionController.hpp"

ProstateRobotMotionController::ProstateRobotMotionController() 
// ProstateRobotMotionController::ProstateRobotMotionController(ProstateRobotMotors *motors) : motors(motors)
{
	// fpga_utils = motors->_fpga_util;
}

// TODO: WIll probably deprecate since each robot method have their own version.
bool ProstateRobotMotionController::Spin()
{
	// Reads the latest encoder value for each motor
	UpdateMotorReadings();
	// if (fpga_utils->IsFootPedalPressed())
	// {
	// 	MoveAllMotors();
	// 	return true;
	// }
	// else
	// {
	// 	UpdateMotorsStallDetect();
	// 	// Stop all the motors of the robot
	// 	StopRobotMotion();
	// 	return false;
	// }
}

// Method for update different robot parameters
void ProstateRobotMotionController::UpdateMotorReadings()
{
	// ProstateRobotMotorMapPtr motors_list = motors->GetMotorsListMap();
	// // Update Motor Variables
	// for (auto motor : motors_list)
	// {
	// 	motor.second->UpdateEncoder();
	// }
}

// Update Motor Variables
void ProstateRobotMotionController::UpdateMotorsStallDetect()
{
	// ProstateRobotMotorMapPtr motors_list = motors->GetMotorsListMap();
	// for(auto motor:motors_list)
	// {
	// 	motor.second->UpdateMotorStallTime();
	// }
}

void ProstateRobotMotionController::StopRobotMotion()
{
	// ProstateRobotMotorMapPtr motors_list = motors->GetMotorsListMap();
	// for (auto motor : motors_list)
	// {
	// 	motor.second->StopMotor();
	// }
}

void ProstateRobotMotionController::MoveAllMotors()
{
	// ProstateRobotMotorMapPtr motors_list = motors->GetMotorsListMap();
	// for (auto motor : motors_list)
	// {
	// 	motor.second->MoveMotor();
	// }
}

void ProstateRobotMotionController::MoveMotor(ProstateRobotMotor motor_name)
{
	// motors->GetMotor(motor_name)->MoveMotor();
	// motors->GetMotorsListMap()[motor_name]->MoveMotor();
}

void ProstateRobotMotionController::StopMotor(ProstateRobotMotor motor_name)
{
	// motors->GetMotor(motor_name)->StopMotor();
}

void ProstateRobotMotionController::DisableAllMotors()
{
	// ProstateRobotMotorMapPtr motors_list = motors->GetMotorsListMap();
	// for (auto motor : motors_list)
	// {
	// 	motor.second->_enabled = false;
	// }
}