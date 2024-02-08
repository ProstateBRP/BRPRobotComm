#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/QR>
#include <string.h>
#include <stdexcept>

#include "Logger.hpp"
#include "ProstateRobotEnumerations.hpp"
namespace polyfit
{
    // Number of coefficients to be calculated for each method
    enum FitType
    {
        LINEAR = 1,
        QUADRATIC = 3,
        CUBIC = 4
    };

    // Projection planes for the needle tip pose (w.r.t robot base)
    enum Plane
    {
        ZX,
        ZY
    };

    // Index for accessing the x,y,z elements for the reported tip vector
    enum Idx
    {
        X = 0,
        Y = 1,
        Z = 2,
    };

    // alpha is the yaw value and beta is the pitch value of the needle pose (w.r.t robot base)
    struct EstimatedAngleOutput
    {
        double alpha;
        double beta;
    };

    class PolyFit
    {
    public:
        // Functions
        EstimatedAngleOutput CalcAngle(const std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>> &);

    private:
        // Functions
        int DetermineOrder(const std::vector<double> &data);
        std::vector<double> ExtractPointVec(Idx);
        double CalculatePlanarAngle(const Eigen::VectorXd &coeffs, Plane plane);
        Eigen::VectorXd PerformLinearFit(const std::vector<double> &, const std::vector<double> &);
        Eigen::VectorXd PerformPolynomialLeastSquaresFit(const std::vector<double> &, const std::vector<double> &);
        double CalculateSlope(const Eigen::VectorXd &);
        double CalculateAngle(const double &, Plane);

        // Attributes
        std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>> data_pts;
    };
}
