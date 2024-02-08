#include "SteeringAlgorithm.h"

SteeringAlgorithm::SteeringAlgorithm(const double &max_curvature) : max_curvature{max_curvature}
{
    tgt_pos_needle_frame_rotated = Eigen::Matrix<double, 4, 1, Eigen::DontAlign>::Zero();
}
/*!
    Calculates the curvature from the needle tip frame to the target point defined in the needle tip frame.
*/
double SteeringAlgorithm::CalcCurvature(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &tgt_position_needle_frame)
{
    return abs(1 / (tgt_position_needle_frame(1) / 2 + pow(tgt_position_needle_frame(2), 2) / (2 * tgt_position_needle_frame(1))));
}

/*!
    Calculate angle of the target as seen in the x-y plane of the needle frame.
    0 radians corresponds with negative y-axis.
    Return value is in radians.
*/
double SteeringAlgorithm::CalcTargetAngle(const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &tgt_pt_needle_frame)
{
    return (atan2(-tgt_pt_needle_frame(0), tgt_pt_needle_frame(1)) + M_PI);
}

/*!
    Checks if the calculated curvature for a given target is feasible.
    curvature: Calculated curvature from the needle tip to the target location.
    Return: 1 if target is reachable, 0 if not.
*/
bool SteeringAlgorithm::isReachable(const double &curvature)
{
    return max_curvature > curvature;
}

double SteeringAlgorithm::CalcInsertionLengthDiff(const double &curr_insertion_len_mm, const double &total_insertion_len_mm)
{
    // Find the remaining insertion distance
    double remaining_insertion_len = abs(total_insertion_len_mm - curr_insertion_len_mm);
    double radius = 1 / curvature;
    // The target is on the y-z plane of the needle tip's frame
    double center_y = -radius;
    double y_prime = tgt_pos_needle_frame_rotated(1) - center_y;
    double arc_angle = atan2(tgt_pos_needle_frame_rotated(2), y_prime);
    double arc_len = abs(arc_angle * radius);
    return abs(arc_len - remaining_insertion_len);
}

/*!
    This methods rotates the needle frame about its z-axis by a desired angle so that the target is placed on the needle frame's yz plane.
*/
Eigen::Matrix<double, 4, 1, Eigen::DontAlign> SteeringAlgorithm::PlaceTargetOnNeedleYzPlane(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &needle_pose_rbt_frame, const Eigen::Matrix<double, 4, 1, Eigen::DontAlign> &tgt_pos_rbt_frame)
{
    // Rotate needle frame so that the target is placed at the y-z plane of the needle frame to enable the calculation of the
    // curvature.
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> rotated_needle_frame = RotateAboutZ(needle_pose_rbt_frame, theta_d);
    // Recalculate the target pos in the rotated needle frame
    return (rotated_needle_frame.inverse() * tgt_pos_rbt_frame);
}

/*!
    This function applies a transformation about the z axis and returns the
    new rotated 4x4 transformation.
*/
Eigen::Matrix<double, 4, 4, Eigen::DontAlign> SteeringAlgorithm::RotateAboutZ(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &trans, const double &theta)
{
    Eigen::Matrix<double, 4, 4, Eigen::DontAlign> rotation_z;
    rotation_z << cos(theta), -sin(theta), 0., 0.,
        sin(theta), cos(theta), 0., 0.,
        0., 0., 1., 0.,
        0., 0., 0., 1;
    return (trans * rotation_z);
}