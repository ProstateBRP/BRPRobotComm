#include "ProstateRobotAutomatedHomingMode.hpp"

ProstateRobotAutomatedHomingMode::ProstateRobotAutomatedHomingMode(ProstateRobotMotionController *motion_ctrl) : ProstateRobotModeBase(motion_ctrl)
{
	SetName(kRobotModes.AUTOMATED_HOMING_MODE);
}

// Method for homing the robot
// TODO: Implement coordinated homing
void ProstateRobotAutomatedHomingMode::Run(const string &current_state)
{
	// if (IsFootPedalPressed())
	// {
	// 	Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// 	Motor *back_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
	// 	Motor *back_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
	// 	Motor *front_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
	// 	Motor *front_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
	// 	if (!insertion->_homed)
	// 	{
	// 		// Retract Insertion before homing the base
	// 		front_left->StopMotor();
	// 		front_right->StopMotor();
	// 		back_left->StopMotor();
	// 		back_right->StopMotor();
	// 		insertion->_homing = true;
	// 		insertion->HomeMotor();
	// 	}
	// 	else
	// 	{
	// 		if (!front_left->_homed || !front_right->_homed || !back_left->_homed || !back_right->_homed)
	// 		{
	// 			if (!front_left->_homed || !back_left->_homed)
	// 			{

	// 				// Send Right side motors to some arbitrarily large positive value
	// 				front_right->_setpoint = kConstants.move_to_right;
	// 				back_right->_setpoint = kConstants.move_to_right;

	// 				// If the front left motor has not been homed
	// 				if (!front_left->_homed)
	// 				{
	// 					// Set homing to true for the web ui and home the motor
	// 					front_left->HomeMotor();
	// 					front_left->_homing = true;
	// 				}
	// 				else
	// 				{
	// 					// If the motor has been homed
	// 					// Stop that axis and ensure no movement by assigning the setpoint to the current position
	// 					front_left->StopMotor();
	// 					front_left->_setpoint = front_left->GetEncoderPositionTicks();
	// 					front_left->_homing = false;
	// 				}

	// 				// If the back left motor has not been homed
	// 				if (!back_left->_homed)
	// 				{
	// 					back_left->HomeMotor();
	// 					back_left->_homing = true;
	// 				}
	// 				else
	// 				{
	// 					back_left->StopMotor();
	// 					back_left->_setpoint = back_left->GetEncoderPositionTicks();
	// 					back_left->_homing = false;
	// 				}
	// 			}

	// 			else if ((front_left->_homed && back_left->_homed) && (!front_right->_homed || !back_right->_homed))
	// 			{

	// 				front_left->_setpoint = front_left->_minTicks;
	// 				back_left->_setpoint = back_left->_minTicks;

	// 				if (!front_right->_homed)
	// 				{
	// 					front_right->HomeMotor();
	// 					front_right->_homing = true;
	// 				}
	// 				else
	// 				{
	// 					front_right->StopMotor();
	// 					front_right->_setpoint = front_right->GetEncoderPositionTicks();
	// 					front_right->_homing = false;
	// 				}

	// 				if (!back_right->_homed)
	// 				{
	// 					back_right->HomeMotor();
	// 					back_right->_homing = true;
	// 				}
	// 				else
	// 				{
	// 					back_right->StopMotor();
	// 					back_right->_setpoint = back_right->GetEncoderPositionTicks();
	// 					back_right->_homing = false;
	// 				}
	// 			}
	// 		}

	// 		// If all the base motors have touched off their limit switches, send them all to their home location
	// 		else
	// 		{
	// 			front_left->_setpoint = front_left->_homeOffsetInTicks;
	// 			front_right->_setpoint = front_right->_homeOffsetInTicks;
	// 			back_left->_setpoint = back_left->_homeOffsetInTicks;
	// 			back_right->_setpoint = back_right->_homeOffsetInTicks;
	// 		}
	// 	}
	// }
	// else
	// {
	// 	RunIdle();
	// }
}

// Method that is used to put the robot back into its home position
void ProstateRobotAutomatedHomingMode::SetMotorSetPointsToHome()
{
	// Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
	// Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// Motor *back_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
	// Motor *back_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
	// Motor *front_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
	// Motor *front_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
	// front_left->_setpoint = 375000;
	// front_right->_setpoint = -375000;
	// back_left->_setpoint = 375000;
	// back_right->_setpoint = -375000;
	// insertion->_setpoint = 0;
	// rotation->_setpoint = 0;
}
