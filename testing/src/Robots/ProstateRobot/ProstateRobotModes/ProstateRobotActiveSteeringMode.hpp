#ifndef __ProstateRobotActiveSteeringMode_HPP_
#define __ProstateRobotActiveSteeringMode_HPP_

#include "ProstateRobotModeBase.hpp"
#include "PolyFit.h"
#include "ProstateRobotConstants.hpp"
#include "BicycleKinematics.h"
#include "CurvSteering.h"
#include "ProstateRobotKinematicsController.hpp"

class ProstateRobotActiveSteeringMode : public ProstateRobotModeBase
{
public:
    ProstateRobotActiveSteeringMode(ProstateRobotMotionController *);
    virtual void Run(const string &current_state = "");

    int CalcVelocityFreq(const double &);
    double CalcVelocityRpm(const double &);
    bool CheckDirectionChange(const double &);
    bool CommandRotationToStop(Motor *);
    void SetAlpha(double alpha) { curv_steering->alpha = alpha; }
    void SetTargetAngle(double value) { curv_steering->theta_d = value; }
    void SetCurvMethod(CurvMethod method) { curv_steering->curv_method = method; }
    void UpdateCurvParams(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &);
    void ActiveCompensation();
    void UpdateRotationDirection(const double &, Motor *);
    const double GetAlpha() { return curv_steering->alpha; }
    const double GetTargetAngle() { return curv_steering->theta_d; }
    double GetRotationMotorPositionUnit();
    int LinearInterpolation(double des_rpm, const vector<double> &, const vector<int> &);
    double ConvertMotorTicksPerSecToRpm(Motor *);

private:
    Timer timer;
    ProstateRobotConstants kConstants;
    uint stop_counter{0}; // Temp refer to https://wpiaimlab.atlassian.net/browse/MRC-59
    double max_rotation_speed_rpm{kConstants.max_rotation_speed_clinical_mode_rpm};
    CurvSteering *curv_steering{nullptr};
    RotationDirection old_dir{RotationDirection::CCW};
    vector<int> frequency_positive_dir;
    vector<int> frequency_negative_dir;
    vector<double> rpm_positive_dir;
    vector<double> rpm_negative_dir;
};

#endif //__ProstateRobotActiveSteeringMode_HPP_