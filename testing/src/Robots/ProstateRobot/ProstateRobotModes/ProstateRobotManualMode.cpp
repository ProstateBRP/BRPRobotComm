#include "ProstateRobotManualMode.hpp"

ProstateRobotManualMode::ProstateRobotManualMode(ProstateRobotMotionController *motion_ctrl) : ProstateRobotModeBase(motion_ctrl)
{
	SetName(kRobotModes.MANUAL_MODE);
	// Experimentally calculated values using frequency sweep mode
	rpm_positive_dir = {0., 0.029113, 0.164148, 0.288959, 0.431078, 0.640218, 0.963437, 1.45948, 2.659549, 5.180603, 11.444549, 13.463671, 20.9551, 27.358274, 36.039252, 45.547515, 58.996934, 78.710358, 105.850173, 141.386226, 188.106219, 247.388042,
						317.394418, 397.785133};

	frequency_positive_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088,
							  1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580,
							  1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};

	rpm_negative_dir = {0, 0.053648, 0.189476, 0.330537, 0.613857, 1.094737, 2.019837, 3.221758, 3.772608, 6.38341, 10.635663, 15.307919, 20.23076, 27.238521, 35.237872, 45.332754, 58.497216, 77.516035, 103.283567, 138.165886, 183.915033, 242.763009, 312.833219, 391.708604};

	frequency_negative_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088, 1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580, 1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};
	// Update controller with the velocity
	// velocity_controller.SetVelSetpoint(des_vel_);

	// Set the desired open-loop velocity to the motor
	Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
	rotation->_velocity_controller.SetVelSetpoint(des_vel_);
	double des_freq = LinearInterpolation(des_vel_, rpm_positive_dir, frequency_positive_dir);
	rotation->_velocity = des_freq;
	timer.tic();
}

void ProstateRobotManualMode::Run(const string &current_state)
{
	// Move motors to their setpoints
	if (IsFootPedalPressed())
	{
		Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
		rotation->_velocity = LinearInterpolation(des_vel_, rpm_positive_dir, frequency_positive_dir);
		motion_ctrl->MoveAllMotors();
	}
	// If the foot pedal is not pressed
	else
	{
		timer.toc();
		RunIdle();
		timer.tic();
	}
}

double ProstateRobotManualMode::ConvertMotorTicksPerSecToRpm(Motor *motor)
{
	return motor->GetEncoderVelocity() * (60 / (motor->_ticksPerUnit * 2 * M_PI)); // Get RPM velocity ;
}

int ProstateRobotManualMode::LinearInterpolation(double des_rpm, const vector<double> &rpm_data, const vector<int> &freq_data)
{
	if (rpm_data.size() != freq_data.size() || rpm_data.empty())
	{
		std::cerr << "Error: Invalid input vectors." << std::endl;
		return 0; // Return a default value or handle the error as needed.
	}

	// Find the two closest x values in the vector
	size_t i = 0;
	while (i < rpm_data.size() - 1 && rpm_data[i] < des_rpm)
	{
		i++;
	}

	if (i == 0)
	{
		i = 1; // Ensure we don't go out of bounds
	}

	// Perform linear interpolation
	double rpm_sample_0 = rpm_data[i - 1];
	double rpm_sample_1 = rpm_data[i];
	int freq_data_sample_0 = freq_data[i - 1];
	int freq_data_sample_1 = freq_data[i];

	// Calculate the interpolated value as a double
	double interpolatedValue = freq_data_sample_0 + ((des_rpm - rpm_sample_0) / (rpm_sample_1 - rpm_sample_0)) * (freq_data_sample_1 - freq_data_sample_0);

	// Convert the interpolated value back to int if needed
	int result = static_cast<int>(interpolatedValue);

	return result;
}

/*
This function can replace the arguments in the "if (IsFootPedalPressed())" block to test constant velocity control.
*/
void ProstateRobotManualMode::ClosedLoopControl()
{
	timer.toc();
	double elapsed_time_sec = timer.ConvertMicrosecToSec(timer.time());
	Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
	double control_output_rpm = rotation->_velocity_controller.CalculateVelInputCommand(ConvertMotorTicksPerSecToRpm(rotation), elapsed_time_sec);
	int commanded_vel = LinearInterpolation(control_output_rpm, rpm_positive_dir, frequency_positive_dir);
	rotation->_velocity = commanded_vel;
	motion_ctrl->MoveAllMotors();
	timer.tic();
}