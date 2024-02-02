#include "ProstateRobotKinematicsController.hpp"

ProstateRobotKinematicsController::ProstateRobotKinematicsController(ProstateRobotMotors *motors, ProstateKinematics *kinematics) : motors(motors), kinematics(kinematics)
{
}

ProstateRobotMotorSetpointMap ProstateRobotKinematicsController::AxisSetpointValidator()
{
    Logger &log = Logger::GetInstance();
    log.Log("========================= AXIS VALIDATION =========================", logger::INFO, false);

    // Get Motors
    Motor *front_left = motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
    Motor *front_right = motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
    Motor *back_left = motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
    Motor *back_right = motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
    Motor *insertion = motors->GetMotor(ProstateRobotMotor::INSERTION);

    // Define Valid Positions
    ProstateRobotMotorSetpointMap valid_setpoint_map;
    valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] = front_left->_setpoint;
    valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->_setpoint;
    valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] = back_left->_setpoint;
    valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] = back_right->_setpoint;
    valid_setpoint_map[ProstateRobotMotor::INSERTION] = insertion->_setpoint;

    /* ================= Legs separation Validations ==========================*/
    double front_legs_separation = front_left->GetSetPointInPositionUnit() - front_right->GetSetPointInPositionUnit();
    if (front_legs_separation > allowed_axial_separation_mm)
    {
        log.Log("Front Axial Separation of " + to_string(front_legs_separation) + " mm  will exceed the maximum allowed separation of " + to_string(allowed_axial_separation_mm) + " mm", logger::WARNING, false);
    }

    double backAxialSeparation = back_left->GetSetPointInPositionUnit() - back_right->GetSetPointInPositionUnit();
    if (backAxialSeparation > allowed_axial_separation_mm)
    {
        log.Log("Back Axial Separation of " + to_string(backAxialSeparation) + " mm  will exceed the maximum allowed separation of " + to_string(allowed_axial_separation_mm) + " mm", logger::WARNING, false);
    }

    /// Check angulation
    ProstateRobotAngulationInput angulation_input;
    angulation_input.front_left = front_left->GetSetPointInPositionUnit();
    angulation_input.front_right = front_right->GetSetPointInPositionUnit();
    angulation_input.back_left = back_left->GetSetPointInPositionUnit();
    angulation_input.back_right = back_right->GetSetPointInPositionUnit();
    AngulationOutput angulation = kinematics->GetAngulation(angulation_input);

    // Check alpha limits
    if (angulation.alpha > max_allowed_yaw_angle_deg)
    {
        log.Log("Alpha angle of " + to_string(angulation.alpha) + " degrees  will exceed the maximum allowed angle of " + to_string(max_allowed_yaw_angle_deg) + " degrees", logger::WARNING, false);
    }
    else if (angulation.alpha < min_allowed_yaw_angle_deg)
    {
        log.Log("Alpha angle of " + to_string(angulation.alpha) + " degrees  will exceed the minimum allowed angle of " + to_string(min_allowed_yaw_angle_deg) + " degrees", logger::WARNING, false);
    }

    // check beta limits
    if (angulation.beta > max_allowed_pitch_angle_deg)
    {
        log.Log("Beta angle of " + to_string(angulation.beta) + " degrees  will exceed the maximum allowed angle of " + to_string(max_allowed_pitch_angle_deg) + " degrees", logger::WARNING, false);
    }
    else if (angulation.beta < min_allowed_pitch_angle_deg)
    {
        log.Log("Beta angle of " + to_string(angulation.beta) + " degrees  will exceed the minimum allowed angle of " + to_string(min_allowed_pitch_angle_deg) + " degrees", logger::WARNING, false);
    }

    /// Front Right
    if (valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] > front_right->_maxTicks)
    {
        log.Log("Front Right Axis of " + to_string(front_right->GetSetPointInPositionUnit()) + " mm  will exceed the maximum allowed distance of " + to_string(front_right->ConvertTicksToPositionUnit(front_right->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->_maxTicks;
    }
    else if (valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] < front_right->_minTicks)
    {
        log.Log("Front Right Axis of " + to_string(front_right->GetSetPointInPositionUnit()) + " mm  will exceed the minimum allowed distance of " + to_string(front_right->ConvertTicksToPositionUnit(front_right->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->_minTicks;
    }

    /// Front Left
    if (valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] > front_left->_maxTicks)
    {
        log.Log("Front Left Axis of " + to_string(front_left->GetSetPointInPositionUnit()) + " mm  will exceed the maximum allowed distance of " + to_string(front_left->ConvertTicksToPositionUnit(front_left->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] = front_left->_maxTicks;
    }
    else if (valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] < front_left->_minTicks)
    {
        log.Log("Front Left Axis of " + to_string(front_left->GetSetPointInPositionUnit()) + " mm  will exceed the minimum allowed distance of " + to_string(front_left->ConvertTicksToPositionUnit(front_left->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] = front_left->_minTicks;
    }

    /// Back Right
    if (valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] > back_right->_maxTicks)
    {
        log.Log("Back Right Axis of " + to_string(back_right->GetSetPointInPositionUnit()) + " mm  will exceed the maximum allowed distance of " + to_string(back_right->ConvertTicksToPositionUnit(back_right->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] = back_right->_maxTicks;
    }
    else if (valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] < back_right->_minTicks)
    {
        log.Log("Back Right Axis of " + to_string(back_right->GetSetPointInPositionUnit()) + " mm  will exceed the minimum allowed distance of " + to_string(back_right->ConvertTicksToPositionUnit(back_right->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] = back_right->_minTicks;
    }

    /// Back Left
    if (valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] > back_left->_maxTicks)
    {
        log.Log("Back Left Axis of " + to_string(back_left->GetSetPointInPositionUnit()) + " mm  will exceed the maximum allowed distance of " + to_string(back_left->ConvertTicksToPositionUnit(back_left->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] = back_left->_maxTicks;
    }
    else if (valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] < back_left->_minTicks)
    {
        log.Log("Back Left Axis of " + to_string(back_left->GetSetPointInPositionUnit()) + " mm  will exceed the minimum allowed distance of " + to_string(back_left->ConvertTicksToPositionUnit(back_left->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] = back_left->_minTicks;
    }

    /// Insertion
    if (valid_setpoint_map[ProstateRobotMotor::INSERTION] > insertion->_maxTicks)
    {
        log.Log("Insertion Axis of " + to_string(insertion->GetSetPointInPositionUnit()) + " mm  will exceed the maximum allowed distance of " + to_string(insertion->ConvertTicksToPositionUnit(insertion->_maxTicks)) + " mm", logger::WARNING, false);

        valid_setpoint_map[ProstateRobotMotor::INSERTION] = insertion->_maxTicks;
    }
    else if (valid_setpoint_map[ProstateRobotMotor::INSERTION] < insertion->_minTicks)
    {
        log.Log("Insertion Axis of " + to_string(insertion->GetSetPointInPositionUnit()) + " mm  will exceed the minimum allowed distance of " + to_string(insertion->ConvertTicksToPositionUnit(insertion->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::INSERTION] = insertion->_minTicks;
    }
    log.Log("============================== END ==========================", logger::INFO, false);

    return valid_setpoint_map;
}

// Calculates reachable target point based on the kinematics constraints of the robot
ProstateRobotMotorSetpointMap ProstateRobotKinematicsController::AxisSetpointValidator(ProstateRobotMotorSetpointMap &input_map)
{

    Logger &log = Logger::GetInstance();
    log.Log("========================= AXIS VALIDATION ====================", logger::INFO, false);

    // Get Motors
    Motor *front_left = motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
    Motor *front_right = motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
    Motor *back_left = motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
    Motor *back_right = motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
    Motor *insertion = motors->GetMotor(ProstateRobotMotor::INSERTION);

    // The input setpoints will be modified if exceeding the robot limits
    ProstateRobotMotorSetpointMap valid_setpoint_map = input_map;

    //------------------------------------------------------------------------
    /// Legs separation Validations
    double front_legs_separation = front_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_LEFT]) - front_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_RIGHT]);
    if (front_legs_separation > allowed_axial_separation_mm)
    {
        log.Log("Front Axial Separation of " + to_string(front_legs_separation) + " mm  will exceed the maximum allowed separation of " + to_string(allowed_axial_separation_mm) + " mm", logger::WARNING, false);
    }

    double backAxialSeparation = back_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_LEFT]) - back_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_RIGHT]);
    if (backAxialSeparation > allowed_axial_separation_mm)
    {
        log.Log("Back Axial Separation of " + to_string(backAxialSeparation) + " mm  will exceed the maximum allowed separation of " + to_string(allowed_axial_separation_mm) + " mm", logger::WARNING, false);
    }

    /// Check angulation
    ProstateRobotAngulationInput angulation_input;
    angulation_input.front_left = front_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_LEFT]);
    angulation_input.front_right = front_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_RIGHT]);
    angulation_input.back_left = back_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_LEFT]);
    angulation_input.back_right = back_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_RIGHT]);
    AngulationOutput angulation = kinematics->GetAngulation(angulation_input);
    // Check alpha limits
    if (angulation.alpha > max_allowed_yaw_angle_deg)
    {
        log.Log("Alpha angle of " + to_string(angulation.alpha) + " degrees  will exceed the maximum allowed angle of " + to_string(max_allowed_yaw_angle_deg) + " degrees", logger::WARNING, false);
    }
    else if (angulation.alpha < min_allowed_yaw_angle_deg)
    {
        log.Log("Alpha angle of " + to_string(angulation.alpha) + " degrees  will exceed the minimum allowed angle of " + to_string(min_allowed_yaw_angle_deg) + " degrees", logger::WARNING, false);
    }

    // check beta limits
    if (angulation.beta > max_allowed_pitch_angle_deg)
    {
        log.Log("Beta angle of " + to_string(angulation.beta) + " degrees  will exceed the maximum allowed angle of " + to_string(max_allowed_pitch_angle_deg) + " degrees", logger::WARNING, false);
    }
    else if (angulation.beta < min_allowed_pitch_angle_deg)
    {
        log.Log("Beta angle of " + to_string(angulation.beta) + " degrees  will exceed the minimum allowed angle of " + to_string(min_allowed_pitch_angle_deg) + " degrees", logger::WARNING, false);
    }

    /// Front Right
    if (input_map[ProstateRobotMotor::FRONT_RIGHT] > front_right->_maxTicks)
    {
        log.Log("Front Right Axis of " + to_string(front_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_RIGHT])) + " mm  will exceed the maximum allowed distance of " + to_string(front_right->ConvertTicksToPositionUnit(front_right->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->_maxTicks;
    }
    else if (input_map[ProstateRobotMotor::FRONT_RIGHT] < front_right->_minTicks)
    {
        log.Log("Front Right Axis of " + to_string(front_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_RIGHT])) + " mm  will exceed the minimum allowed distance of " + to_string(front_right->ConvertTicksToPositionUnit(front_right->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->_minTicks;
    }

    /// Front Left
    if (input_map[ProstateRobotMotor::FRONT_LEFT] > front_left->_maxTicks)
    {
        log.Log("Front Left Axis of " + to_string(front_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_LEFT])) + " mm  will exceed the maximum allowed distance of " + to_string(front_left->ConvertTicksToPositionUnit(front_left->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] = front_left->_maxTicks;
    }
    else if (input_map[ProstateRobotMotor::FRONT_LEFT] < front_left->_minTicks)
    {
        log.Log("Front Left Axis of " + to_string(front_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::FRONT_LEFT])) + " mm  will exceed the minimum allowed distance of " + to_string(front_left->ConvertTicksToPositionUnit(front_left->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::FRONT_LEFT] = front_left->_minTicks;
    }

    /// Back Right
    if (input_map[ProstateRobotMotor::BACK_RIGHT] > back_right->_maxTicks)
    {
        log.Log("Back Right Axis of " + to_string(back_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_RIGHT])) + " mm  will exceed the maximum allowed distance of " + to_string(back_right->ConvertTicksToPositionUnit(back_right->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] = back_right->_maxTicks;
    }
    else if (input_map[ProstateRobotMotor::BACK_RIGHT] < back_right->_minTicks)
    {
        log.Log("Back Right Axis of " + to_string(back_right->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_RIGHT])) + " mm  will exceed the minimum allowed distance of " + to_string(back_right->ConvertTicksToPositionUnit(back_right->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_RIGHT] = back_right->_minTicks;
    }

    /// Back Left
    if (input_map[ProstateRobotMotor::BACK_LEFT] > back_left->_maxTicks)
    {
        log.Log("Back Left Axis of " + to_string(back_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_LEFT])) + " mm  will exceed the maximum allowed distance of " + to_string(back_left->ConvertTicksToPositionUnit(back_left->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] = back_left->_maxTicks;
    }
    else if (input_map[ProstateRobotMotor::BACK_LEFT] < back_left->_minTicks)
    {
        log.Log("Back Left Axis of " + to_string(back_left->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::BACK_LEFT])) + " mm  will exceed the minimum allowed distance of " + to_string(back_left->ConvertTicksToPositionUnit(back_left->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::BACK_LEFT] = back_left->_minTicks;
    }

    /// Insertion
    if (input_map[ProstateRobotMotor::INSERTION] > insertion->_maxTicks)
    {
        log.Log("Insertion Axis of " + to_string(insertion->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::INSERTION])) + " mm  will exceed the maximum allowed distance of " + to_string(insertion->ConvertTicksToPositionUnit(insertion->_maxTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::INSERTION] = insertion->_maxTicks;
    }
    else if (input_map[ProstateRobotMotor::INSERTION] < insertion->_minTicks)
    {
        log.Log("Insertion Axis of " + to_string(insertion->ConvertTicksToPositionUnit(input_map[ProstateRobotMotor::INSERTION])) + " mm  will exceed the minimum allowed distance of " + to_string(insertion->ConvertTicksToPositionUnit(insertion->_minTicks)) + " mm", logger::WARNING, false);
        valid_setpoint_map[ProstateRobotMotor::INSERTION] = insertion->_minTicks;
    }
    log.Log("============================== END ==========================", logger::INFO, false);

    return valid_setpoint_map;
}

ProstateRobotMotorSetpointMap ProstateRobotKinematicsController::CalculateKinematicallyValidSetpoints(const Eigen::Matrix<double, 4, 4, Eigen::DontAlign> &base_to_desired_target_robot_coord)
{
    // Get Motors
    Motor *front_left = motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
    Motor *front_right = motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
    Motor *back_left = motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
    Motor *back_right = motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
    Motor *insertion = motors->GetMotor(ProstateRobotMotor::INSERTION);
    Motor *rotation = motors->GetMotor(ProstateRobotMotor::ROTATION);
    // Run IK
    Prostate_IK_outputs ik_output = kinematics->InverseKinematics(base_to_desired_target_robot_coord);
    // Convert the IK output from their corresponding units to ticks
    ProstateRobotMotorSetpointMap ik_setpoints_map;
    ik_setpoints_map[ProstateRobotMotor::FRONT_LEFT] = front_left->ConvertPositionUnitToTicks(ik_output.front_left_slider);
    ik_setpoints_map[ProstateRobotMotor::FRONT_RIGHT] = front_right->ConvertPositionUnitToTicks(ik_output.front_right_slider);
    ik_setpoints_map[ProstateRobotMotor::BACK_LEFT] = back_left->ConvertPositionUnitToTicks(ik_output.back_left_slider);
    ik_setpoints_map[ProstateRobotMotor::BACK_RIGHT] = back_right->ConvertPositionUnitToTicks(ik_output.back_right_slider);
    ik_setpoints_map[ProstateRobotMotor::INSERTION] = insertion->ConvertPositionUnitToTicks(ik_output.zInsertion);
    // Run Axis validation
    ProstateRobotMotorSetpointMap validAxisSetpoints = AxisSetpointValidator(ik_setpoints_map);
    return validAxisSetpoints;
}

Eigen::Matrix<double, 4, 4, Eigen::DontAlign> ProstateRobotKinematicsController::GetNeedleGuidePoseRobotCoord()
{
    ProstateRobotAngulationInput angulation_input;
    Motor *front_left = motors->GetMotor(ProstateRobotMotor::FRONT_LEFT);
    Motor *front_right = motors->GetMotor(ProstateRobotMotor::FRONT_RIGHT);
    Motor *back_left = motors->GetMotor(ProstateRobotMotor::BACK_LEFT);
    Motor *back_right = motors->GetMotor(ProstateRobotMotor::BACK_RIGHT);
    angulation_input.front_left = front_left->GetEncoderPositionUnit();
    angulation_input.front_right = front_right->GetEncoderPositionUnit();
    angulation_input.back_left = back_left->GetEncoderPositionUnit();
    angulation_input.back_right = back_right->GetEncoderPositionUnit();

    return kinematics->GetNeedleGuidePoseRobotCoord(angulation_input).BaseToTreatment;
}
