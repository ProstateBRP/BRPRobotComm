#ifndef __ProstateRobotClinicalMode_HPP_
#define __ProstateRobotClinicalMode_HPP_

#include "ProstateRobotModeBase.hpp"

#include "PolyFit.h"
#include "ProstateRobotConstants.hpp"
// #include "BicycleKinematics.h"
#include "CurvSteering.h"
#include "ProstateRobotKinematicsController.hpp"

class ProstateRobotClinicalMode : public ProstateRobotModeBase
{
public:
    ProstateRobotClinicalMode(ProstateRobotMotionController *, ProstateRobotKinematicsController *);

    const bool isRetractingNeedle() { return retracting_needle; }
    int CalcVelocityFreq(const double &);
    double CalcVelocityRpm(const double &);
    double ConvertMotorTicksPerSecToRpm();
    // double ConvertMotorTicksPerSecToRpm(Motor *);
    bool hasReachedTarget(double epsilon = 1e-1);
    bool isInTargetingPos(double orientation_tol = 1e-2, double pos_tol = 1e-3);
    bool IsSetpointListEmpty(int);
    bool isNeedleAtHome();
    bool CheckDirectionChange(const double &);
    // bool CommandRotationToStop(Motor *);
    void Run(const std::string &);
    // void Run(const std::string &, std::queue<Motor*> &);
    void CleanUp();
    // void SetAlpha(double alpha) { curv_steering->alpha = alpha; }
    void RetractNeedle();
    // void SetCurvMethod(CurvMethod method) { curv_steering->curv_method = method; }
    void UpdateCurvParams(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &);
    void SaveNeedleTipPose();
    void ActiveCompensation();
    void MoveMotorsTargeting();
    void MoveOneMotorTargeting(int);
    void PrepareNeedleRetract();
    void UpdateInsertionLength();
    void UpdateRotationDirection();
    // void UpdateRotationDirection(const double &, Motor *);
    void SetBaseToTreatmentRobotCoord(Eigen::Matrix<double, 4, 4, Eigen::DontAlign> *matrix) { base_to_treatment_robot_coord = matrix; }
    void SetBaseToDesiredTargetRobotCoord(Eigen::Matrix<double, 4, 4, Eigen::DontAlign> *matrix) { base_to_desired_target_robot_coord = matrix; }
    void PushBackKinematicTipAsActualPose();
    void PushBackActualNeedlePosAndUpdatePose(const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &);
    void UpdateCurvParamsAndInsertionLength();
    void UpdateNeedleTipPositionBicycleKinematic(const double &, const double &);
    const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> GetBaseToTreatmentRobotCoordKinematic();
    // const double GetAlpha() { return curv_steering->alpha; }
    // const double GetTargetAngle() { return curv_steering->theta_d; }
    double GetRotationMotorPositionUnit();
    int LinearInterpolation(double des_rpm, const vector<double> &, const vector<int> &);
    void SetBaseToTreatmentRobotCoordKinematic(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);

private:
    ProstateRobotConstants kConstants;
    uint stop_counter{0}; // Temp refer to https://wpiaimlab.atlassian.net/browse/MRC-59
    ProstateRobotStates robot_state;
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> *base_to_treatment_robot_coord{nullptr};
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> *base_to_desired_target_robot_coord{nullptr};
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> base_to_treatment_robot_coord_kinematic;
    bool retracting_needle{false};
    double max_rotation_speed_rpm{kConstants.max_rotation_speed_clinical_mode_rpm};
    vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>> actual_tip_positions;
    polyfit::PolyFit poly_fit;
    Timer timer;
    CurvSteering *curv_steering{nullptr};
    // RotationDirection old_dir{RotationDirection::CCW};
    // BicycleKinematics bicycle_kinematics;
    ProstateRobotKinematicsController *kinematics_ctrl{nullptr};
    vector<int> frequency_positive_dir;
    vector<int> frequency_negative_dir;
    vector<double> rpm_positive_dir;
    vector<double> rpm_negative_dir;
    mutable std::mutex mutex; // To ensure threadsafe behavior
};

#endif //__ProstateRobotClinicalMode_HPP_