#include "ProstateRobotClinicalMode.hpp"

ProstateRobotClinicalMode::ProstateRobotClinicalMode(ProstateRobotMotionController *motion_ctrl, ProstateRobotKinematicsController *kinematics_ctrl) : ProstateRobotModeBase(motion_ctrl), kinematics_ctrl(kinematics_ctrl), curv_steering(new CurvSteering(kConstants.max_curvature))
{
	SetName(kRobotModes.CLINICAL_MODE);
    // Experimentally calculated values using frequency sweep mode
    rpm_positive_dir = {0., 0.029113, 0.164148, 0.288959, 0.431078, 0.640218, 0.963437, 1.45948, 2.659549, 5.180603, 11.444549, 13.463671, 20.9551, 27.358274, 36.039252, 45.547515, 58.996934, 78.710358, 105.850173, 141.386226, 188.106219, 247.388042,
                        317.394418, 397.785133};

    frequency_positive_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088,
                              1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580,
                              1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};

    rpm_negative_dir = {0, 0.053648, 0.189476, 0.330537, 0.613857, 1.094737, 2.019837, 3.221758, 3.772608, 6.38341, 10.635663, 15.307919, 20.23076, 27.238521, 35.237872, 45.332754, 58.497216, 77.516035, 103.283567, 138.165886, 183.915033, 242.763009, 312.833219, 391.708604};

    frequency_negative_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088, 1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580, 1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};
	// Start the timer
	timer.tic();
}

void ProstateRobotClinicalMode::Run(const std::string &current_state){}

// void ProstateRobotClinicalMode::Run(const std::string &current_state, std::queue<Motor*> &motors_queue)
// {
// 	timer.toc();
// 	Logger &log = Logger::GetInstance();
// 	if (current_state == robot_state.TARGETING)
// 	{
// 		int reached_motor = 0;
// 		bool syn_motion = true;
// 		if (IsFootPedalPressed())
// 		{
// 			// Only move base (no insertion and rotation)
// 			if (syn_motion)
// 			{
// 				Motor *motor = motors_queue.front();
// 				// stop all motors
// 				motion_ctrl->StopRobotMotion();

// 				if (motor->IsMotorWithinSetpointBounds())
// 				{
// 					log.Log("Motor " + motor->_name + " reached setpoint.", logger::INFO, true);
// 					if (!motor->_setpointList.empty())
// 					{
// 						motor->_setpointList.pop();
// 					}
// 					log.Log("Setpoint List Length: " + to_string(motor->_setpointList.size()), logger::INFO, true);
// 					if (!motor->_setpointList.empty())
// 					{
// 						motor->_setpoint = motor->_setpointList.front();
// 					}
// 					else
// 					{
// 						log.Log("Setpoint List is empty.", logger::INFO, true);
// 						motor->StopMotor();
// 						motor->DisableMotor();
// 					}

// 					motors_queue.push(motor);
// 					motors_queue.pop();
// 				}
// 				else
// 				{
// 					motor->MoveMotor();
// 				}
// 			}
// 			else
// 			{
// 				if (!isInTargetingPos())
// 				{
// 					// TODO: Move the needle tip to be flushed with the needle guide. https://wpiaimlab.atlassian.net/browse/MRC-50
// 					MoveMotorsTargeting();
// 				}
// 				else
// 				{
// 					motion_ctrl->StopRobotMotion();
// 					motion_ctrl->DisableAllMotors();
// 				}
// 			}
			
// 		}
// 		else
// 		{ // Update Motor Stall Detect
// 			RunIdle();
// 		}
// 	}
// 	else if (current_state == robot_state.MOVE_TO_TARGET)
// 	{
// 		if (IsFootPedalPressed())
// 		{
// 			if (isRetractingNeedle())
// 			{
// 				RetractNeedle();
// 			}
// 			else if (!hasReachedTarget())
// 			{
// 				ActiveCompensation();
// 			}
// 			else
// 			{
// 				motion_ctrl->StopRobotMotion();
// 				motion_ctrl->DisableAllMotors();
// 			}
// 		}
// 		else
// 		{
// 			RunIdle();
// 		}
// 	}
// 	else
// 	{
// 		// Update Motor Stall Detect
// 		RunIdle();
// 	}
// 	timer.tic();
// }

void ProstateRobotClinicalMode::ActiveCompensation()
{
	// Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);

	// double insertion_old = insertion->GetEncoderLastPositionUnit();
	// double rotation_old = rotation->GetEncoderLastPositionUnit();

	// double w_hat = curv_steering->CalcRotationalVel(rotation->GetEncoderPositionUnit());
	// double desired_vel_rpm = CalcVelocityRpm(w_hat);
	// // Use abs to remove the sign for controller input calculation (the reported vel in motor space is unsigned)
	// rotation->_velocity_controller.SetVelSetpoint(abs(desired_vel_rpm));
	// // Calculate commanded velocity based on observed velocity and time elapsed. Multiplying by signbit brings back the original velocity sign.
	// double elapsed_time_sec = timer.ConvertMicrosecToSec(timer.time());
	// double commanded_vel_rpm = rotation->_velocity_controller.CalculateVelInputCommand(ConvertMotorTicksPerSecToRpm(rotation), elapsed_time_sec) * (std::signbit(w_hat) ? -1 : 1);
	// rotation->_velocity = CalcVelocityFreq(commanded_vel_rpm);
	// // Check for direction change and send three stop commands to the FPGA. NOTE: Will be refactored once Charles enables auto direction change detection in the FPGA level.
	// if (CheckDirectionChange(w_hat))
	// {
	// 	if (CommandRotationToStop(rotation))
	// 	{
	// 		old_dir = (old_dir == RotationDirection::CCW) ? RotationDirection::CW : RotationDirection::CCW;
	// 	}
	// }
	// // If direction has not changed keep sending the setpoints which in turn will determine the table value (rotation dir).
	// else
	// {
	// 	UpdateRotationDirection(w_hat, rotation);
	// 	rotation->MoveMotor();
	// }
	// insertion->MoveMotor();

	// double du1 = insertion->GetEncoderPositionUnit() - insertion_old;
	// double du2 = rotation->GetEncoderPositionUnit() - rotation_old;
	// // Update the bicycle kinematic
	// UpdateNeedleTipPositionBicycleKinematic(du1, du2);
}

bool ProstateRobotClinicalMode::CheckDirectionChange(const double &commanded_w_hat)
{
	// if ((commanded_w_hat < 0 && old_dir == RotationDirection::CW) || (commanded_w_hat > 0 && old_dir == RotationDirection::CCW))
	// {
	// 	return true;
	// }
	// else
	// {
	// 	return false;
	// }
}

// bool ProstateRobotClinicalMode::CommandRotationToStop(Motor *rotation)
// {
// 	// Send 5 consecutive stop commands to the motor
// 	// TODO: Charles will enable direction change detection on the card
// 	if (stop_counter < 10)
// 	{
// 		stop_counter++;
// 	}
// 	else
// 	{
// 		stop_counter = 0;
// 		return true;
// 	}
// 	rotation->StopMotor();
// 	return false;
// }

void ProstateRobotClinicalMode::UpdateRotationDirection(){}

// void ProstateRobotClinicalMode::UpdateRotationDirection(const double &commanded_w_hat, Motor *rotation)
// {
// 	if (commanded_w_hat < 0)
// 	{
// 		rotation->_setpoint = -1e6;
// 		old_dir = RotationDirection::CCW;
// 	}
// 	else
// 	{
// 		rotation->_setpoint = 1e6;
// 		old_dir = RotationDirection::CW;
// 	}
// }

int ProstateRobotClinicalMode::CalcVelocityFreq(const double &des_vel_rpm)
{
	// Find the desired motor frequency for the negative rotation dir (refer to the robot's stick fig for +/- rotation dir)
	if (des_vel_rpm <= 0)
	{
		return LinearInterpolation(abs(des_vel_rpm), rpm_negative_dir, frequency_negative_dir);
	}
	else
	{
		return LinearInterpolation(des_vel_rpm, rpm_positive_dir, frequency_positive_dir);
	}
}

void ProstateRobotClinicalMode::UpdateNeedleTipPositionBicycleKinematic(const double &du1, const double &du2)
{
	// Will result in NaN if both are zeros
	if (du1 == 0 && du2 == 0)
	{
		return;
	}
	// SetBaseToTreatmentRobotCoordKinematic(bicycle_kinematics.ForwardKinematicsBicycleModel(GetBaseToTreatmentRobotCoordKinematic(), du1, du2));
}

void ProstateRobotClinicalMode::MoveMotorsTargeting()
{
	motion_ctrl->MoveMotor(ProstateRobotMotor::BACK_LEFT);
	motion_ctrl->MoveMotor(ProstateRobotMotor::BACK_RIGHT);
	motion_ctrl->MoveMotor(ProstateRobotMotor::FRONT_LEFT);
	motion_ctrl->MoveMotor(ProstateRobotMotor::FRONT_RIGHT);
}

void ProstateRobotClinicalMode::MoveOneMotorTargeting(int motor_id)
{
	// if (motor_id == 0)
	// {
	// 	motion_ctrl->MoveMotor(ProstateRobotMotor::BACK_LEFT);
	// 	Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);

	// 	Logger &log = Logger::GetInstance();
	// 	log.Log("Setpoint List Length: " + to_string(motor->_setpointList.size()), logger::INFO, true);
	// 	log.Log("Setpoint List: ", logger::INFO, true);
	// 	for (auto it = motor->_setpointList.front(); it != motor->_setpointList.back(); it++)
	// 	{
	// 		log.Log(to_string(it), logger::INFO, true);
	// 	}
	// }
	// else if (motor_id == 1)
	// {
	// 	motion_ctrl->MoveMotor(ProstateRobotMotor::BACK_RIGHT);
	// }
	// else if (motor_id == 2)
	// {
	// 	motion_ctrl->MoveMotor(ProstateRobotMotor::FRONT_LEFT);
	// }
	// else if (motor_id == 3)
	// {
	// 	motion_ctrl->MoveMotor(ProstateRobotMotor::FRONT_RIGHT);
	// }
}

void ProstateRobotClinicalMode::RetractNeedle()
{
	// Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// motion_ctrl->MoveMotor(ProstateRobotMotor::INSERTION);
	// if (insertion->IsLimit())
	// {
	// 	retracting_needle = false;
	// 	insertion->StopMotor();
	// 	insertion->DisableMotor();
	// }
	// double insertion_old = insertion->GetEncoderLastPositionUnit();
	// double du1 = insertion->GetEncoderPositionUnit() - insertion_old;
	// // Update the bicycle kinematic
	// UpdateNeedleTipPositionBicycleKinematic(du1, 0);
}

void ProstateRobotClinicalMode::PrepareNeedleRetract()
{
	// // Disable all motors
	// motion_ctrl->DisableAllMotors();
	// // Enable insertion motor and retract until it hits the limit.
	// Motor *insertion_motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// insertion_motor->_enabled = true;
	// insertion_motor->_homing = true;
	// insertion_motor->_setpoint = insertion_motor->_homeOffsetInTicks;
	// // Enable tool retract flag
	// retracting_needle = true;
}

void ProstateRobotClinicalMode::UpdateCurvParams(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &target_vector)
{
	curv_steering->CalcCurvParams(GetBaseToTreatmentRobotCoordKinematic(), target_vector, GetRotationMotorPositionUnit());
}

double ProstateRobotClinicalMode::GetRotationMotorPositionUnit()
{
	// Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
	// return rotation->GetEncoderPositionUnit();
}

void ProstateRobotClinicalMode::PushBackActualNeedlePosAndUpdatePose(const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &reported_tip_pos)
{
	Logger &log = Logger::GetInstance();
	// Convert reported tip position from the robot base frame to needle guide frame
	// Eigen::Matrix<double, 4, 1, Eigen::DontAlign> reported_tip_pos_needle_guide_coord = kinematics_ctrl->GetNeedleGuidePoseRobotCoord().inverse() * Eigen::Matrix<double, 4, 1, Eigen::DontAlign>(reported_tip_pos(0), reported_tip_pos(1), reported_tip_pos(2), 1);
	// Push back the reported tip position
	// actual_tip_positions.push_back(reported_tip_pos);
	// try
	// {
	// 	// estimate rotation angle about
	// 	polyfit::EstimatedAngleOutput estimated_angle = poly_fit.CalcAngle(actual_tip_positions);
	// 	// Log estimated rotation angles about the needle base for the needle tip pose
	// 	string ss{"Alpha,  " + to_string(estimated_angle.alpha * 180 / M_PI) + " degrees."};
	// 	log.Log(ss, logger::INFO, true);
	// 	ss.clear();
	// 	ss = "Beta,  " + to_string(estimated_angle.beta * 180 / M_PI) + " degrees.";
	// 	log.Log(ss, logger::INFO, true);
	// 	ss = "Theta,  " + to_string(GetRotationMotorPositionUnit() * 180 / M_PI) + " degrees.";
	// 	log.Log(ss, logger::INFO, true);
	// 	// Update the current pose based on the estimated pose
	// 	// Orientation Component
	// 	std::cout << "Needle Pose before rbt coord: \n"
	// 			  << GetBaseToTreatmentRobotCoordKinematic() << std::endl;

	// 	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> new_base_to_treatment_rbt_coord;
	// 	new_base_to_treatment_rbt_coord.setIdentity();

	// 	new_base_to_treatment_rbt_coord = bicycle_kinematics.ApplyRotationFixedAngles(new_base_to_treatment_rbt_coord, Eigen::Matrix<double, 3, 1, Eigen::DontAlign>(estimated_angle.beta, estimated_angle.alpha, GetRotationMotorPositionUnit()));
	// 	// Position Component
	// 	new_base_to_treatment_rbt_coord(0, 3) = reported_tip_pos(0);
	// 	new_base_to_treatment_rbt_coord(1, 3) = reported_tip_pos(1);
	// 	new_base_to_treatment_rbt_coord(2, 3) = reported_tip_pos(2);
	// 	SetBaseToTreatmentRobotCoordKinematic(new_base_to_treatment_rbt_coord);
	// 	std::cout << "Needle Pose after rbt coord: \n"
	// 			  << GetBaseToTreatmentRobotCoordKinematic() << std::endl;
	// }
	// catch (const std::exception &e)
	// {
	// 	// Log the error
	// 	Logger &log = Logger::GetInstance();
	// 	std::string log_msg{e.what()};
	// 	log.Log(log_msg, logger::ERROR, true);
	// }
}

/*
 Initializes the kinematic tip's transformation (base to the needle tip) used during needle insertion in move_to_target state.
*/
void ProstateRobotClinicalMode::SaveNeedleTipPose()
{
	SetBaseToTreatmentRobotCoordKinematic((*base_to_treatment_robot_coord));
}

void ProstateRobotClinicalMode::PushBackKinematicTipAsActualPose()
{
	actual_tip_positions.clear();
	// The first point is considered as the needle guide's frame location.
	Eigen::Matrix<double, 4, 4, 2> needle_guide_pose = kinematics_ctrl->GetNeedleGuidePoseRobotCoord();
	Eigen::Matrix<double, 3, 1, Eigen::DontAlign> tip_pose;
	tip_pose << needle_guide_pose(0, 3), needle_guide_pose(1, 3), needle_guide_pose(2, 3);
	actual_tip_positions.push_back(tip_pose);
}

void ProstateRobotClinicalMode::CleanUp()
{
	actual_tip_positions.clear();
}

void ProstateRobotClinicalMode::UpdateCurvParamsAndInsertionLength()
{
	// Calculate new curvature, steering effort, and desired theta
	Eigen::Matrix<double, 4, 1, Eigen::DontAlign> target_pt_vector = Eigen::Matrix<double, 4, 1, Eigen::DontAlign>((*base_to_desired_target_robot_coord)(0, 3), (*base_to_desired_target_robot_coord)(1, 3), (*base_to_desired_target_robot_coord)(2, 3), 1);
	UpdateCurvParams(target_pt_vector);
	// Update needle insertion's setpoint
	UpdateInsertionLength();
}

void ProstateRobotClinicalMode::UpdateInsertionLength()
{
	// Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// double total_insertion_setpoint_mm = insertion->GetSetPointInPositionUnit();
	// double insertion_diff_mm = curv_steering->CalcInsertionLengthDiff(insertion->GetEncoderPositionUnit(), total_insertion_setpoint_mm);
	// double updated_total_insertion_mm = total_insertion_setpoint_mm + insertion_diff_mm;
	// insertion->_setpoint = insertion->ConvertPositionUnitToTicks(updated_total_insertion_mm);
}

bool ProstateRobotClinicalMode::isInTargetingPos(double orientation_tol, double pos_tol)
{
	// // Get the motors
	// Motor *front_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
	// Motor *front_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
	// Motor *back_left = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
	// Motor *back_right = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);

	// //  Check if all four legs are at their setpoint
	// if (front_left->IsMotorWithinSetpointBounds() && front_right->IsMotorWithinSetpointBounds() && back_left->IsMotorWithinSetpointBounds() && back_right->IsMotorWithinSetpointBounds())
	// {
	// 	return true;
	// }

	// // The robot has reached the targeting position
	// return false;
}

bool ProstateRobotClinicalMode::IsSetpointListEmpty(int motor_id)
{
	// Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
	// if (motor_id == 0)
	// {
	// 	Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
	// }
	// else if (motor_id ==1)
	// {
	// 	Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
	// }
	// else if (motor_id == 2)
	// {
	// 	Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
	// }
	// else if (motor_id == 3)
	// {
	// 	Motor *motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
	// }
	// // log length of setpoint list
	// Logger &log = Logger::GetInstance();
	// log.Log("Setpoint List Length: " + to_string(motor->_setpointList.size()), logger::INFO, true);
	// if (motor->_setpointList.empty())
	// {
	// 	return true;
	// }
	return false;
}

bool ProstateRobotClinicalMode::hasReachedTarget(double epsilon)
{
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> kinematic_tip_pos_rbt_coord = GetBaseToTreatmentRobotCoordKinematic();
	return (abs(kinematic_tip_pos_rbt_coord(2, 3) - (*base_to_desired_target_robot_coord)(2, 3)) < epsilon);
}

bool ProstateRobotClinicalMode::isNeedleAtHome()
{
	// Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
	// return insertion->IsLimit();
}

int ProstateRobotClinicalMode::LinearInterpolation(double des_rpm, const vector<double> &rpm_data, const vector<int> &freq_data)
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
void ProstateRobotClinicalMode::SetBaseToTreatmentRobotCoordKinematic(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &matrix)
{
	std::lock_guard<std::mutex> lock(mutex); // Lock the mutex to ensure exclusive access
	base_to_treatment_robot_coord_kinematic = matrix;
}
const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ProstateRobotClinicalMode::GetBaseToTreatmentRobotCoordKinematic()
{
	std::lock_guard<std::mutex> lock(mutex); // Lock the mutex before reading
	return base_to_treatment_robot_coord_kinematic;
}

double ProstateRobotClinicalMode::CalcVelocityRpm(const double &w_hat)
{
	if (round(w_hat * 1000) / 1000 == 0.0)
	{
		return 0;
	}
	return (w_hat * max_rotation_speed_rpm);
}

double ProstateRobotClinicalMode::ConvertMotorTicksPerSecToRpm(){}

// double ProstateRobotClinicalMode::ConvertMotorTicksPerSecToRpm(Motor *motor)
// {
// 	return motor->GetEncoderVelocity() * (60 / (motor->_ticksPerUnit * 2 * M_PI)); // Get RPM velocity ;
// }

/*======================================================*/
// ==================== Legacy Code ====================
/*======================================================*/

// Will not work for the angulated insertions
// bool ProstateRobotClinicalMode::isInTargetingPos(double orientation_tol, double pos_tol)
// {
// 	// Check the orientation
// 	if (!base_to_treatment_robot_coord->block(0, 0, 3, 3).isApprox(base_to_desired_target_robot_coord->block(0, 0, 3, 3), orientation_tol))
// 	{
// 		return false;
// 	}
// 	//  Check the x and y of the robot
// 	if (!base_to_treatment_robot_coord->block(0, 3, 2, 1).isApprox(base_to_desired_target_robot_coord->block(0, 3, 2, 1), pos_tol))
// 	{
// 		return false;
// 	}

// 	// The robot has reached the targeting position
// 	return true;
// }