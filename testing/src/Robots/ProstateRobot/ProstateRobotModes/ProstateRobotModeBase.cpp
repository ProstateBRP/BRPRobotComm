#include "ProstateRobotModeBase.hpp"

ProstateRobotModeBase::ProstateRobotModeBase(ProstateRobotMotionController *motion_ctrl) : motion_ctrl(motion_ctrl)
{
}

bool ProstateRobotModeBase::IsFootPedalPressed()
{
	return motion_ctrl->fpga_utils->IsFootPedalPressed();
}

/* Update Motor Stall Detect */
void ProstateRobotModeBase::RunIdle()
{
	motion_ctrl->UpdateMotorsStallDetect();
	motion_ctrl->StopRobotMotion();
}

void ProstateRobotModeBase::Run(const string &current_state)
{
	// Move motors to their setpoints
	if (IsFootPedalPressed())
	{
		motion_ctrl->MoveAllMotors();
	}
	// If the foot pedal is not pressed
	else
	{
		RunIdle();
	}
}
