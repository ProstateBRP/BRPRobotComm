//============================================================================
// Name        : Robot.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is strictly for defining an Abstract Robot Class
//				 all children of this type must implement the following
//============================================================================

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <memory>

#include "Eigen/Dense"

#include "Timer.hpp"
#include "Logger.hpp"

class Robot
{
public:
	//================ Constructor/Destructors ================
	Robot(){};
	virtual ~Robot(){};
	//================ Parameters =================
	// An object of type robot must have the following parameters
	string _name;							  // Name of the given robot
	string _mode;							  // Current robot mode
	string _socketIGTConnection{""};		  // IGT Connection Status
	string current_state{""};				  // Stores a string regarding the current state of the robot
	bool FlagCalibration{false};			  // Flag for when a registration is set
	bool FlagTarget{false};					  // Flag for when a target point is set
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> registration;			  // Transformation between the imager and the robot's reference frame
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> current_pose_image_coord; // Transformation that represents current location of the treatment zone
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> robot_base_to_zframe;
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> target_full_pose_image_coord; // Target pose as sent from the navigation
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> base_to_treatment_robot_coord;
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> base_to_desired_target_robot_coord;
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> reachable_target_pose_imager_coord; // Represents reachable target by the robot's treatment zone in RAS

	//================ Public Methods =================
	bool GetCalibrationFlag() { return FlagCalibration; }
	string GetCurrentState() { return current_state; }
	bool GetTargetFlag() { return FlagTarget; }
	void SetTargetFlag(bool flag) { FlagTarget = flag; }
	void SetCalibrationFlag(bool flag) { FlagCalibration = flag; }
	void SetCurrentPoseImageCoord(Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &matrix) { current_pose_image_coord = matrix; }
	const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetCurrentPositionImageCoord() { return current_pose_image_coord; }
	const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetRegistration() { return registration; }
	const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetReachableTargetImageCoord() { return reachable_target_pose_imager_coord; }
	const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetTargetImageCoord() { return target_full_pose_image_coord; }
	//================ Pure Virtual Methods =================
	virtual bool isHomed() = 0;
	virtual bool CheckForStalls() = 0;
	virtual void Reset() = 0;
	virtual void Update() = 0;
	virtual void StopRobot() = 0;
	virtual void ZeroRobot() = 0;
	virtual void SetRegistration(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &) = 0;
	virtual void UpdateBaseToTarget() = 0;
	virtual void SetTargetImageCoord(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &) = 0;
	virtual void RunInverseKinematics() = 0;
	virtual void SetTargetPointPosVectorImageCoord(const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &) = 0;
	virtual vector<string> GetRobotModesList() = 0;
	virtual Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ConvertFromImagerToRobotBase(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &) = 0;
	virtual Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ConvertFromRobotBaseToImager(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &) = 0;
	virtual Eigen::Matrix<double, 3, 1, Eigen::DontAlign> GetTargetPointPosVectorImageCoord() = 0;
};

#endif /* ROBOT_HPP_ */
