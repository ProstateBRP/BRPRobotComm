//============================================================================
// Name        : ProstateKinematics.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#ifndef PROSTATEKINEMATICS_HPP_
#define PROSTATEKINEMATICS_HPP_

#ifndef RADIAN_TO_DEGREE
#define RADIAN_TO_DEGREE (57.29578)
#endif

#include "../Kinematics.hpp"

#include <iostream>
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "Eigen/Dense"

// Parameters for the biopsy needle
struct BiopsyNeedle
{
	int _needleGauge;
	double _needleLength;
	double _bevelAngle;
};

struct ProstateRobotForwardKinematicsInput
{
	double front_left;
	double front_right;
	double back_left;
	double back_right;
	double insertion;
};

struct Prostate_FK_outputs
{
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> BaseToTreatment;
	double xNeedleTip;
	double yNeedleTip;
	double zNeedleTip;
};

struct Prostate_IK_outputs
{
	double front_left_slider;
	double front_right_slider;
	double back_left_slider;
	double back_right_slider;
	double zInsertion;
	double zRotation;
};

struct NeedleHolder
{
	double _holderLength;
	double _needleBaseToHolderTip;
	double _holderBaseToRobotBaseZ;
};

// Refer to https://doi.org/10.1002/rcs.1671
struct AngulationOutput
{
	double alpha; // Yaw angle
	double beta;  // Pitch angle
};

struct ProstateRobotAngulationInput
{
	double front_left;
	double front_right;
	double back_left;
	double back_right;
};

class ProstateKinematics : public Kinematics
{

public:
	ProstateKinematics();
	ProstateKinematics(BiopsyNeedle *biopsy_needle);

	// Inputs to the Forward and Inverse  Kinematics are given in units, not ticks
	Prostate_FK_outputs ForwardKinematics(const ProstateRobotForwardKinematicsInput &);
	Prostate_IK_outputs InverseKinematics(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &TargetPose);

	// Specialized method which calculates alpha and beta value given axis values
	Prostate_FK_outputs GetNeedleGuidePoseRobotCoord(const ProstateRobotAngulationInput &);
	AngulationOutput GetAngulation(const ProstateRobotAngulationInput &);
	void UpdateNeedleLength();

	// Robot Specific Parameters
	double _lengthTrapSideLink;
	double _widthTrapTop;
	double _heightLowerTrapOffset;
	double _heightUpperTrapOffset;
	double _lengthNeedleTipOffset;
	double _distanceBetweenTraps;
	double _baseToNeedleGuideZ;

	// Needle holder specifications
	NeedleHolder _needleHolder;
	// Biopsy needle specifications
	BiopsyNeedle *biopsy_needle;
	//**Values that update with motion**//
	// Trapazoid points of rotation
	double _xFrontPointOfRotation;
	double _yFrontPointOfRotation;
	double _zFrontPointOfRotation;

	double _xRearPointOfRotation;
	double _yRearPointOfRotation;
	double _zRearPointOfRotation;

	// Angulation Variables
	double _C;
	double _h;
};

#endif /* PROSTATEKINEMATICS_HPP_ */
