//============================================================================
// Name        : ProstateKinematics.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#include "ProstateKinematics.hpp"

ProstateKinematics::ProstateKinematics()
{
}

ProstateKinematics::ProstateKinematics(BiopsyNeedle *biopsy_needle)
{

	// Biopsy needle parameters
	this->biopsy_needle = biopsy_needle;
	// {_holderLength, _needleBaseToHolderTip, _holderBaseToRobotBaseZ}
	_needleHolder = {90., 0., 90.};
	// Robot Specific Parameters
	_lengthTrapSideLink = 124.;	   // L
	_widthTrapTop = 84.;		   // B
	_heightLowerTrapOffset = 12.;  // H1
	_heightUpperTrapOffset = 67.5; // H2
	UpdateNeedleLength();		   // Z-offset from F_base to needle tip (value will change upon receiving needle length from slicer)
	_distanceBetweenTraps = 181.5; // D
	_baseToNeedleGuideZ = 223.4;   // Distance from base to needle guide tip at home
	//**Values that update with motion**//
	// Trapezoid points of rotation
	_xFrontPointOfRotation = 0.;
	_yFrontPointOfRotation = 0.;
	_zFrontPointOfRotation = 0.;

	_xRearPointOfRotation = 0.;
	_yRearPointOfRotation = 0.;
	_zRearPointOfRotation = 0.;

	// Angulation Variable
	_C = 15; //old 0. C is the distance between the point of rotation and center of the front trapezoid stage in the
	_h = 0.; // h is the distance between the needle's direction and center of the front trapezoid stage in the vertical direction.
}

Prostate_FK_outputs ProstateKinematics::ForwardKinematics(const ProstateRobotForwardKinematicsInput &fk_input_map)

{
	double front_left_slider = fk_input_map.front_left;
	double front_right_slider = fk_input_map.front_right;
	double back_left_slider = fk_input_map.back_left;
	double back_right_slider = fk_input_map.back_right;
	double zInsertion = fk_input_map.insertion;

	Prostate_FK_outputs FK;

	//*** BASE FORWARD KINEMATICS ***//
	_xFrontPointOfRotation = (front_left_slider + front_right_slider) / 2;

	double yF_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yF_2 = pow(_lengthTrapSideLink, 2);
	double yF_3 = pow((front_left_slider - front_right_slider - _widthTrapTop) / 2, 2);
	_yFrontPointOfRotation = yF_1 + sqrt(yF_2 - yF_3);

	_zFrontPointOfRotation = -_C;

	_xRearPointOfRotation = (back_left_slider + back_right_slider) / 2;
	double yR_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yR_2 = pow(_lengthTrapSideLink, 2);
	double yR_3 = pow((back_left_slider - back_right_slider - _widthTrapTop) / 2, 2);
	_yRearPointOfRotation = yR_1 + sqrt(yR_2 - yR_3);

	_zRearPointOfRotation = 0;

	// Alpha is the yaw angle of the needle tip relative to an imaginary straight insertion line.
	double _alpha = atan2(_xFrontPointOfRotation - _xRearPointOfRotation, _distanceBetweenTraps);
	// Beta is the pitch angle of the needle tip relative to an imaginary straight insertion line.
	double _beta = atan2(_yRearPointOfRotation - _yFrontPointOfRotation, _distanceBetweenTraps);
	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTipYaw;
	rotationBaseToTipYaw << cos(_alpha), 0, sin(_alpha),
		0, 1, 0,
		-sin(_alpha), 0, cos(_alpha);

	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTipPitch;
	rotationBaseToTipPitch << 1, 0, 0,
		0, cos(_beta), -sin(_beta),
		0, sin(_beta), cos(_beta);

	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTip;
	rotationBaseToTip = rotationBaseToTipYaw * rotationBaseToTipPitch;

	FK.xNeedleTip = ((_lengthNeedleTipOffset + zInsertion) * cos(_beta) * sin(_alpha)) + (_h * sin(_beta) * sin(_alpha)) + _xFrontPointOfRotation;

	FK.yNeedleTip = (_h * cos(_beta)) - ((_lengthNeedleTipOffset + zInsertion) * sin(_beta)) + _yFrontPointOfRotation;

	//*** NEEDLE DRIVER FORWARD KINEMATICS ***//
	FK.zNeedleTip = ((_lengthNeedleTipOffset + zInsertion) * cos(_beta) * cos(_alpha)) + (_h * sin(_beta) * cos(_alpha)) + _zFrontPointOfRotation;

	FK.BaseToTreatment << 1, 0, 0, FK.xNeedleTip,
		0, 1, 0, FK.yNeedleTip,
		0, 0, 1, FK.zNeedleTip,
		0, 0, 0, 1;
	FK.BaseToTreatment.block(0, 0, 3, 3) = rotationBaseToTip;

	return FK;
}

// This method takes two points and calculates the base and needle insertion values
Prostate_IK_outputs ProstateKinematics::InverseKinematics(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &TargetPose)
{
	Prostate_IK_outputs IK;

	// Calculate Alpha and Beta values based on the target pose
	double _alpha = atan2(-TargetPose(2, 0), TargetPose(0, 0));
	double _beta = atan2(-TargetPose(1, 2), TargetPose(1, 1));

	// Calculate the insertion value
	IK.zInsertion = ((TargetPose(2, 3) - (_h * sin(_beta) * cos(_alpha))) / (cos(_beta) * cos(_alpha))) - _lengthNeedleTipOffset;

	// Calculate the coordinate of the front point of rotation
	double xFrontPointOfRotationDesired = TargetPose(0, 3) - (((_lengthNeedleTipOffset + IK.zInsertion) * cos(_beta) * sin(_alpha)) + (_h * sin(_beta) * sin(_alpha)));

	double yFrontPointOfRotationDesired = TargetPose(1, 3) - ((_h * cos(_beta)) - ((_lengthNeedleTipOffset + IK.zInsertion) * sin(_beta)));
	// Calculating front right and left slider amounts
	double frontHeight = yFrontPointOfRotationDesired - (_heightLowerTrapOffset + _heightUpperTrapOffset);
	double x1_f = 2 * xFrontPointOfRotationDesired;												 // Front Left Leg
	double x2_f = (2 * sqrt(pow(_lengthTrapSideLink, 2) - pow(frontHeight, 2))) + _widthTrapTop; // Front Right Leg
	// Solving a 2 equation 2 unknown using A*X = B-> X = A.inv * B
	Eigen::Matrix2Xd rightHandSide_f(2, 1);
	rightHandSide_f << x1_f,
		x2_f;
	Eigen::Matrix2d coefficientMatrix;
	coefficientMatrix << 1, 1,
		1, -1;
	Eigen::Matrix2Xd solutionMatrix_f(2, 1);
	solutionMatrix_f = coefficientMatrix.inverse() * rightHandSide_f;
	IK.front_left_slider = solutionMatrix_f(0, 0);
	IK.front_right_slider = solutionMatrix_f(1, 0);

	// Calculate the coordinate of the rear point of rotation
	double xRearPointOfRotationDesired = TargetPose(0, 3) - (((_lengthNeedleTipOffset + IK.zInsertion + _distanceBetweenTraps) * cos(_beta) * sin(_alpha)) + (_h * sin(_beta) * sin(_alpha)));

	double yRearPointOfRotationDesired = TargetPose(1, 3) - ((_h * cos(_beta)) - ((_lengthNeedleTipOffset + IK.zInsertion + _distanceBetweenTraps) * sin(_beta)));
	// Calculating front right and left slider amounts
	double rearHeight = yRearPointOfRotationDesired - (_heightLowerTrapOffset + _heightUpperTrapOffset);
	double x1_r = 2 * xRearPointOfRotationDesired;												// Front Left Leg
	double x2_r = (2 * sqrt(pow(_lengthTrapSideLink, 2) - pow(rearHeight, 2))) + _widthTrapTop; // Front Right Leg
	// Solving a 2 equation 2 unknown using A*X = B-> X = A.inv * B
	Eigen::Matrix2Xd rightHandSide_r(2, 1);
	rightHandSide_r << x1_r,
		x2_r;
	Eigen::Matrix2Xd solutionMatrix_r(2, 1);
	solutionMatrix_r = coefficientMatrix.inverse() * rightHandSide_r;
	IK.back_left_slider = solutionMatrix_r(0, 0);
	IK.back_right_slider = solutionMatrix_r(1, 0);

	IK.zRotation = 0; // FROM OPENIGTLINK TRACKING
	return IK;
}

AngulationOutput ProstateKinematics::GetAngulation(const ProstateRobotAngulationInput &angulation_input)
{
	double front_left_slider = angulation_input.front_left;
	double front_right_slider = angulation_input.front_right;
	double back_left_slider = angulation_input.back_left;
	double back_right_slider = angulation_input.back_right;

	AngulationOutput angulation_output;

	_xFrontPointOfRotation = (front_left_slider + front_right_slider) / 2;

	double yF_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yF_2 = pow(_lengthTrapSideLink, 2);
	double yF_3 = pow((front_left_slider - front_right_slider - _widthTrapTop) / 2, 2);
	_yFrontPointOfRotation = yF_1 + sqrt(yF_2 - yF_3);

	_zFrontPointOfRotation = -_C;

	_xRearPointOfRotation = (back_left_slider + back_right_slider) / 2;
	double yR_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yR_2 = pow(_lengthTrapSideLink, 2);
	double yR_3 = pow((back_left_slider - back_right_slider - _widthTrapTop) / 2, 2);
	_yRearPointOfRotation = yR_1 + sqrt(yR_2 - yR_3);

	// Alpha is the yaw angle of the needle tip relative to an imaginary straight insertion line.
	double _alpha = atan2(_xFrontPointOfRotation - _xRearPointOfRotation, _distanceBetweenTraps);
	_alpha *= RADIAN_TO_DEGREE;
	angulation_output.alpha = _alpha;
	// Beta is the pitch angle of the needle tip relative to an imaginary straight insertion line.
	double _beta = atan2(_yRearPointOfRotation - _yFrontPointOfRotation, _distanceBetweenTraps);
	_beta *= RADIAN_TO_DEGREE;
	angulation_output.beta = _beta;

	return angulation_output;
}

void ProstateKinematics::UpdateNeedleLength()
{
	this->_lengthNeedleTipOffset = this->biopsy_needle->_needleLength +
								   _needleHolder._holderLength - _needleHolder._needleBaseToHolderTip -
								   _needleHolder._holderBaseToRobotBaseZ;
}

Prostate_FK_outputs ProstateKinematics::GetNeedleGuidePoseRobotCoord(const ProstateRobotAngulationInput &angulation_input)
{
	double front_left_slider = angulation_input.front_left;
	double front_right_slider = angulation_input.front_right;
	double back_left_slider = angulation_input.back_left;
	double back_right_slider = angulation_input.back_right;

	Prostate_FK_outputs FK;

	//*** BASE FORWARD KINEMATICS ***//
	_xFrontPointOfRotation = (front_left_slider + front_right_slider) / 2;

	double yF_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yF_2 = pow(_lengthTrapSideLink, 2);
	double yF_3 = pow((front_left_slider - front_right_slider - _widthTrapTop) / 2, 2);
	_yFrontPointOfRotation = yF_1 + sqrt(yF_2 - yF_3);

	_zFrontPointOfRotation = -_C;

	_xRearPointOfRotation = (back_left_slider + back_right_slider) / 2;
	double yR_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yR_2 = pow(_lengthTrapSideLink, 2);
	double yR_3 = pow((back_left_slider - back_right_slider - _widthTrapTop) / 2, 2);
	_yRearPointOfRotation = yR_1 + sqrt(yR_2 - yR_3);

	_zRearPointOfRotation = 0;

	// Alpha is the yaw angle of the needle tip relative to an imaginary straight insertion line.
	double _alpha = atan2(_xFrontPointOfRotation - _xRearPointOfRotation, _distanceBetweenTraps);
	// Beta is the pitch angle of the needle tip relative to an imaginary straight insertion line.
	double _beta = atan2(_yRearPointOfRotation - _yFrontPointOfRotation, _distanceBetweenTraps);
	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTipYaw;
	rotationBaseToTipYaw << cos(_alpha), 0, sin(_alpha),
		0, 1, 0,
		-sin(_alpha), 0, cos(_alpha);

	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTipPitch;
	rotationBaseToTipPitch << 1, 0, 0,
		0, cos(_beta), -sin(_beta),
		0, sin(_beta), cos(_beta);

	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotationBaseToTip;
	rotationBaseToTip = rotationBaseToTipYaw * rotationBaseToTipPitch;

	FK.xNeedleTip = (_baseToNeedleGuideZ * cos(_beta) * sin(_alpha)) + (_h * sin(_beta) * sin(_alpha)) + _xFrontPointOfRotation;

	FK.yNeedleTip = (_h * cos(_beta)) - (_baseToNeedleGuideZ * sin(_beta)) + _yFrontPointOfRotation;

	//*** NEEDLE DRIVER FORWARD KINEMATICS ***//
	FK.zNeedleTip = (_baseToNeedleGuideZ * cos(_beta) * cos(_alpha)) + (_h * sin(_beta) * cos(_alpha)) + _zFrontPointOfRotation;

	FK.BaseToTreatment << 1, 0, 0, FK.xNeedleTip,
		0, 1, 0, FK.yNeedleTip,
		0, 0, 1, FK.zNeedleTip,
		0, 0, 0, 1;
	FK.BaseToTreatment.block(0, 0, 3, 3) = rotationBaseToTip;

	return FK;
}
