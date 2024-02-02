#ifndef _ProstateRobotConstants_HPP_
#define _ProstateRobotConstants_HPP_

struct ProstateRobotConstants
{
    const double max_curvature{0.0022};
    const double max_rotation_speed_clinical_mode_rpm{120}; // Output shaft rotation vel; previous -> 180
    const double robot_base_to_zframe_y_offset = 167.4;
    const double robot_base_to_zframe_z_offset = 239.71;
    const int move_to_left {500000};
    const int move_to_right {-500000};
};

#endif //_ProstateRobotConstants_HPP_