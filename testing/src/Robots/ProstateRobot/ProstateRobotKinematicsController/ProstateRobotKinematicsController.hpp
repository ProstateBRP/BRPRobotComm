#ifndef __ProstateRobotKinematicsController_HPP_
#define __ProstateRobotKinematicsController_HPP_

#include "ProstateRobotMotors.hpp"
#include "ProstateKinematics.hpp"

class ProstateRobotKinematicsController
{
public:
    ProstateRobotKinematicsController();
    // ProstateRobotKinematicsController(ProstateRobotMotors *, ProstateKinematics *);
    ProstateRobotMotorSetpointMap AxisSetpointValidator();
    ProstateRobotMotorSetpointMap AxisSetpointValidator(ProstateRobotMotorSetpointMap &);
    ProstateRobotMotorSetpointMap CalculateKinematicallyValidSetpoints(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetNeedleGuidePoseRobotCoord();

private:
    ProstateRobotMotors *motors{nullptr};
    // ProstateKinematics *kinematics;
    float allowed_axial_separation_mm{308}; // Maximum allowable leg separation distance
    float max_allowed_yaw_angle_deg{10};    // Maximum allowable alpha angle
    float min_allowed_yaw_angle_deg{-10};   // Minimum allowable alpha angle
    float max_allowed_pitch_angle_deg{10};  // Maximum allowable beta angle
    float min_allowed_pitch_angle_deg{-10}; // Minimum allowable beta angle
};

#endif //__ProstateRobotKinematicsController_HPP_