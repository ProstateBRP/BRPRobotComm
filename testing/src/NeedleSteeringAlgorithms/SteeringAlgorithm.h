#ifndef _STEERINGALGORITHM_H_
#define _STEERINGALGORITHM_H_

#include <Eigen/Dense>
#include "Logger.hpp"
#include "Timer.hpp"

class SteeringAlgorithm
{
public:
    SteeringAlgorithm(const double &);
    // Member functions
    double CalcCurvature(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &);
    double CalcTargetAngle(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &);
    bool isReachable(const double &);
    void SetMaxCurvature(const double &max_curvature) { this->max_curvature = max_curvature; }
    double CalcInsertionLengthDiff(const double &, const double &);
    Eigen::Matrix<double, 4, 1, Eigen::DontAlign> PlaceTargetOnNeedleYzPlane(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &, const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RotateAboutZ(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &, const double &);
    // Member attributes
    Eigen::Matrix<double, 4, 1, Eigen::DontAlign> tgt_pos_needle_frame_rotated;
    double curvature{0.};
    double max_curvature{0.};
    double theta_d{0};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
#endif /*_STEERINGALGORITHM_H_*/