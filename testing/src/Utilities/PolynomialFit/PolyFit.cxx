#include "PolyFit.h"

using namespace polyfit;

// Calculate the coefficients of a kth order polynomial given a set of data points
EstimatedAngleOutput PolyFit::CalcAngle(const std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>> &data_pts)
{
    // Determine the fit type based on the data size
    if (data_pts.size() < 2)
    {
        throw(std::invalid_argument("Number of data points MUST be greater than 1!"));
    }
    else
    {
        this->data_pts.clear();
        this->data_pts = data_pts;
        // *** NOTE: The first point is always the position of the needle guide ****
        // Perform linear fit for the first time a tip position is reported.
        if (data_pts.size() == 2)
        {
            // Perform fit for the Z-X plane
            std::vector<double> x_data = ExtractPointVec(Z);
            std::vector<double> y_data = ExtractPointVec(X);
            Eigen::VectorXd zx_coeffs;
            zx_coeffs = PerformLinearFit(x_data, y_data);
            // Perform fit for the Z-Y plane
            y_data.clear();
            y_data = ExtractPointVec(Y);
            Eigen::VectorXd zy_coeffs;
            zy_coeffs = PerformLinearFit(x_data, y_data);
            // Now determine the angle
            EstimatedAngleOutput estimated_angle;
            estimated_angle.alpha = CalculatePlanarAngle(zx_coeffs, Plane::ZX);
            estimated_angle.beta = CalculatePlanarAngle(zy_coeffs, Plane::ZY);
            return estimated_angle;
        }
        else
        {
            // Perform fit for the Z-X plane
            std::vector<double> x_data = ExtractPointVec(Z);
            std::vector<double> y_data = ExtractPointVec(X);
            Eigen::VectorXd zx_coeffs;
            zx_coeffs = PerformPolynomialLeastSquaresFit(x_data, y_data);
            // Perform fit for the Z-Y plane
            y_data.clear();
            y_data = ExtractPointVec(Y);
            Eigen::VectorXd zy_coeffs;
            zy_coeffs = PerformPolynomialLeastSquaresFit(x_data, y_data);
            // Now determine the angle
            EstimatedAngleOutput estimated_angle;
            estimated_angle.alpha = CalculatePlanarAngle(zx_coeffs, Plane::ZX);
            estimated_angle.beta = CalculatePlanarAngle(zy_coeffs, Plane::ZY);
            return estimated_angle;
        }
    }
}

Eigen::VectorXd PolyFit::PerformLinearFit(const std::vector<double> &x_data, const std::vector<double> &y_data)
{
    // Calculate the slope and push back

    if ((x_data.at(1) - x_data.at(0) == 0.))
    {
        throw(runtime_error("****Fit Failed!*** reason: Division-by-zero-Linear-Fit"));
    }

    // For linear fit the coefficient is the slope of the line
    Eigen::VectorXd coeff(1);
    coeff(0) = (y_data.at(1) - y_data.at(0)) / (x_data.at(1) - x_data.at(0));
    return coeff;
}

Eigen::VectorXd PolyFit::PerformPolynomialLeastSquaresFit(const std::vector<double> &x_data, const std::vector<double> &y_data)
{
    // Create Matrix Placeholder of size n x k, n= number of data points,
    // order = order of polynomial(default = Quadratic for when we have 2 reported tips and Cubic for more than 2 reported tips)
    int order = DetermineOrder(x_data);
    // Least Squares regression: V = T.a => result = a = (T' . T)^-1 . T' . V
    Eigen::MatrixXd T(x_data.size(), order);
    Eigen::VectorXd V = Eigen::VectorXd::Map(&y_data.front(), y_data.size());
    Eigen::VectorXd result;

    // Populate the matrix
    for (size_t i = 0; i < x_data.size(); ++i)
    {
        for (size_t j = 0; j < order; ++j)
        {
            T(i, j) = pow(x_data.at(i), j);
        }
    }
    // Solve for linear least square fit
    result = T.householderQr().solve(V);
    return result;
}
double PolyFit::CalculatePlanarAngle(const Eigen::VectorXd &coeffs, Plane plane)
{
    double slope = CalculateSlope(coeffs);
    return CalculateAngle(slope, plane);
}

double PolyFit::CalculateSlope(const Eigen::VectorXd &coeffs)
{
    // Determine the order of the coefficients
    int order = coeffs.size();
    // Linear fit (one coeffs)
    if (order == LINEAR)
    {
        return coeffs(0);
    }
    // Quadratic fit (3 coeffs)
    else if (order == QUADRATIC)
    {
        // Derivative of a Quadratic equation yields the slope of a tangent line at the last reported point
        // Quadratic polynomial form: y = (a2 * x^2) + (a1 * x) + a0 ;
        std::vector<double> x_data = ExtractPointVec(Z);
        double x = x_data.back();
        return (2 * coeffs(2) * x) + coeffs(1);
    }
    // Cubic fit (4 coeffs)
    else
    {
        // Derivative of a Cubic equation yields the slope of a tangent line at the last reported point
        // Cubic polynomial form: y = (a3 * x^3) + (a2 * x^2) + (a1 * x) + a0 ;
        std::vector<double> x_data = ExtractPointVec(Z);
        double x = x_data.back();
        return (3 * coeffs(3) * pow(x, 2)) + (2 * coeffs(2) * x) + coeffs(1);
    }
}
double PolyFit::CalculateAngle(const double &slope, Plane plane)
{
    return (plane == Plane::ZY ? -atan(slope) : atan(slope));
}

std::vector<double> PolyFit::ExtractPointVec(Idx idx)
{
    std::vector<double> extracted_vector{};
    for (auto v : data_pts)
    {
        extracted_vector.push_back(v(idx));
    }
    return extracted_vector;
}

int PolyFit::DetermineOrder(const std::vector<double> &data)
{
    if (data.size() == 3)
    {
        return QUADRATIC;
    }
    else if (data.size() > 3)
    {
        return CUBIC;
    }
}

// EstimatedAngleOutput PolyFit::CalcAngle(const std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>> &);
// {
//     EstimatedAngleOutput output;
//     output.beta = CalcAngleZxPlane();
//     output.omega = -CalcAngleZyPlane(); // Due to frame assignment
//     return output;
// }

// double PolyFit::CalcAngleZxPlane()
// {
//     std::vector<double> x_data = ExtractPointVec(Z);
//     // Linear fit angle calculation
//     if (data_pts.size() == 2)
//     {
//         return atan(zx_linear_coeff.slope);
//     }
//     // Cubic fit angle calculation
//     else
//     {
//         // Calculate the derivative to find the slope at the last point
//         // y' = (3 * a3 * x^2) + (2 * a2 * x) + (a1);
//         double x = x_data.back();
//         double slope = (3 * zx_cubic_coeff.a3 * pow(x, 2)) + (2 * zx_cubic_coeff.a2 * x) + zx_cubic_coeff.a1;
//         return atan(slope);
//     }
// }

// double PolyFit::CalcAngleZyPlane()
// {
//     std::vector<double> x_data = ExtractPointVec(Z);
//     // Linear fit angle calculation
//     if (data_pts.size() == 2)
//     {
//         return atan(zy_linear_coeff.slope);
//     }
//     // Cubic fit angle calculation
//     else
//     {
//         // Calculate the derivative to find the slope at the last point
//         // y' = (3 * a3 * x^2) + (2 * a2 * x) + (a1);
//         double x = x_data.back();
//         double slope = (3 * zy_cubic_coeff.a3 * pow(x, 2)) + (2 * zy_cubic_coeff.a2 * x) + zy_cubic_coeff.a1;
//         return atan(slope);
//     }
// }
