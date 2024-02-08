#ifndef _BICYCLEKINEMATICS_H_
#define _BICYCLEKINEMATICS_H_

#include <iostream>
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "Eigen/Dense"
#include "ProstateRobotConstants.hpp"

class BicycleKinematics
{
public:
    // Constructor
    BicycleKinematics();

    // Member functions
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ForwardKinematicsBicycleModel(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &, const double &, const double &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ApplyRotationEulerAngles(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &, const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ApplyRotationFixedAngles(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &, const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> CalcSpecialEuclideanMatrix(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &);
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> CalcSpecialOrthagonalMatrix(const Eigen::Matrix<double, 3, 3, Eigen::DontAlign> &);
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> ConvertToSpecialOrthagonalVector(const Eigen::Matrix<double, 3, 3, Eigen::DontAlign> &);
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ConvertToSpecialEuclideanMatrix(const Eigen::VectorXd &);
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ConvertToSpecialOrthagonalMatrix(const Eigen::Matrix<double, 3, 1, Eigen::DontAlign> &);

    // Member attributes
    ProstateRobotConstants kConstants;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> e1;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> e2;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> e3;
    double l2;
    double max_curvature;
    Eigen::VectorXd v1;
    Eigen::VectorXd v2;
};

#endif /*_BICYCLEKINEMATICS_H_*/