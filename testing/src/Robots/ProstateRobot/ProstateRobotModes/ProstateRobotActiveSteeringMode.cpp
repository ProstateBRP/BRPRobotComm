#include "ProstateRobotActiveSteeringMode.hpp"

ProstateRobotActiveSteeringMode::ProstateRobotActiveSteeringMode(ProstateRobotMotionController *motion_ctrl) : ProstateRobotModeBase(motion_ctrl), curv_steering(new CurvSteering(kConstants.max_curvature))
{
    SetName(kRobotModes.ACTIVE_STEERING_MODE);
    // Experimentally calculated values using frequency sweep mode
    rpm_positive_dir = {0., 0.029113, 0.164148, 0.288959, 0.431078, 0.640218, 0.963437, 1.45948, 2.659549, 5.180603, 11.444549, 13.463671, 20.9551, 27.358274, 36.039252, 45.547515, 58.996934, 78.710358, 105.850173, 141.386226, 188.106219, 247.388042,
                        317.394418, 397.785133};

    frequency_positive_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088,
                              1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580,
                              1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};

    rpm_negative_dir = {0, 0.053648, 0.189476, 0.330537, 0.613857, 1.094737, 2.019837, 3.221758, 3.772608, 6.38341, 10.635663, 15.307919, 20.23076, 27.238521, 35.237872, 45.332754, 58.497216, 77.516035, 103.283567, 138.165886, 183.915033, 242.763009, 312.833219, 391.708604};

    frequency_negative_dir = {1196612456, 1192549228, 1188486000, 1184422772, 1180359544, 1176296316, 1172233088, 1168169860, 1164106632, 1160043404, 1155980176, 1151916948, 1147853720, 1143790492, 1139727264, 1135664036, 1131600808, 1127537580, 1123474352, 1119411124, 1115347896, 1111284668, 1107221440, 1103158212};
    // Start the timer
    timer.tic();
}

void ProstateRobotActiveSteeringMode::Run(const std::string &current_state)
{

    if (IsFootPedalPressed())
    {
        timer.toc();
        ActiveCompensation();
        timer.tic();
    }
    else
    {
        timer.toc();
        RunIdle();
        timer.tic();
    }
}

void ProstateRobotActiveSteeringMode::ActiveCompensation()
{
    Motor *insertion = motion_ctrl->motors->GetMotor(ProstateRobotMotor::INSERTION);
    Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
    double insertion_old = insertion->GetEncoderLastPositionUnit();
    double rotation_old = rotation->GetEncoderLastPositionUnit();
    double w_hat = curv_steering->CalcRotationalVel(rotation->GetEncoderPositionUnit());
    double desired_vel_rpm = CalcVelocityRpm(w_hat);
    // Profile desired angular velocity
    Logger &logger = Logger::GetInstance();
    string msg = {"Desired Vel: " + to_string(desired_vel_rpm)};
    logger.Log(msg, logger::INFO, false);
    // Use abs to remove the sign for controller input calculation (the reported vel in motor space is unsigned)
    rotation->_velocity_controller.SetVelSetpoint(abs(desired_vel_rpm));
    // Calculate commanded velocity based on observed velocity and time elapsed. Multiplying by signbit brings back the original velocity sign.
    double elapsed_time_sec = timer.ConvertMicrosecToSec(timer.time());
    double commanded_vel_rpm = rotation->_velocity_controller.CalculateVelInputCommand(ConvertMotorTicksPerSecToRpm(rotation), elapsed_time_sec) * (std::signbit(w_hat) ? -1 : 1);
    rotation->_velocity = CalcVelocityFreq(commanded_vel_rpm);
    // Check for direction change and send 10 stop commands to the FPGA. NOTE: Will be refactored once Charles enables auto direction change detection in the FPGA level.
    if (CheckDirectionChange(w_hat))
    {
        if (CommandRotationToStop(rotation))
        {
            old_dir = (old_dir == RotationDirection::CCW) ? RotationDirection::CW : RotationDirection::CCW;
        }
    }
    // If direction has not changed keep sending the setpoints which in turn will determine the table value (rotation dir).
    else
    {
        UpdateRotationDirection(w_hat, rotation);
        rotation->MoveMotor();
    }
    insertion->MoveMotor();

    msg.clear();
    // Profile measured angular velocity
    msg = "Measured Vel: " + to_string((std::signbit(w_hat) ? -1 : 1) * ConvertMotorTicksPerSecToRpm(rotation));
    logger.Log(msg, logger::INFO, false);
    // Profile angle and time
    msg.clear();
    msg = "Theta is: " + to_string(rotation->GetEncoderPositionUnit() * 180 / M_PI);
    logger.Log(msg, logger::INFO, false);
    msg.clear();
    msg = "Time elapsed is: " + to_string(timer.ConvertMicrosecToSec(timer.time()));
    logger.Log(msg, logger::INFO, false);
}

bool ProstateRobotActiveSteeringMode::CheckDirectionChange(const double &commanded_w_hat)
{
    if ((commanded_w_hat < 0 && old_dir == RotationDirection::CW) || (commanded_w_hat > 0 && old_dir == RotationDirection::CCW))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ProstateRobotActiveSteeringMode::CommandRotationToStop(Motor *rotation)
{
    // Send 5 consecutive stop commands to the motor
    // TODO: Charles will enable direction change detection on the card
    if (stop_counter < 10)
    {
        stop_counter++;
    }
    else
    {
        stop_counter = 0;
        return true;
    }
    rotation->StopMotor();
    return false;
}

void ProstateRobotActiveSteeringMode::UpdateRotationDirection(const double &commanded_w_hat, Motor *rotation)
{
    if (commanded_w_hat < 0)
    {
        rotation->_setpoint = -1e6;
        old_dir = RotationDirection::CCW;
    }
    else
    {
        rotation->_setpoint = 1e6;
        old_dir = RotationDirection::CW;
    }
}
double ProstateRobotActiveSteeringMode::CalcVelocityRpm(const double &w_hat)
{
    if (round(w_hat * 1000) / 1000 == 0.0)
    {
        return 0;
    }
    return (w_hat * max_rotation_speed_rpm);
}

int ProstateRobotActiveSteeringMode::CalcVelocityFreq(const double &des_vel_rpm)
{
    // Find the desired motor frequency for the negative rotation dir (refer to the robot's stick fig for +/- rotation dir)
    if (des_vel_rpm <= 0)
    {
        return LinearInterpolation(abs(des_vel_rpm), rpm_negative_dir, frequency_negative_dir);
    }
    else
    {
        return LinearInterpolation(des_vel_rpm, rpm_positive_dir, frequency_positive_dir);
    }
}

double ProstateRobotActiveSteeringMode::GetRotationMotorPositionUnit()
{
    Motor *rotation = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
    return rotation->GetEncoderPositionUnit();
}

int ProstateRobotActiveSteeringMode::LinearInterpolation(double des_rpm, const vector<double> &rpm_data, const vector<int> &freq_data)
{
    if (rpm_data.size() != freq_data.size() || rpm_data.empty())
    {
        std::cerr << "Error: Invalid input vectors." << std::endl;
        return 0; // Return a default value or handle the error as needed.
    }

    // Find the two closest x values in the vector
    size_t i = 0;
    while (i < rpm_data.size() - 1 && rpm_data[i] < des_rpm)
    {
        i++;
    }

    if (i == 0)
    {
        i = 1; // Ensure we don't go out of bounds
    }

    // Perform linear interpolation
    double rpm_sample_0 = rpm_data[i - 1];
    double rpm_sample_1 = rpm_data[i];
    int freq_data_sample_0 = freq_data[i - 1];
    int freq_data_sample_1 = freq_data[i];

    // Calculate the interpolated value as a double
    double interpolatedValue = freq_data_sample_0 + ((des_rpm - rpm_sample_0) / (rpm_sample_1 - rpm_sample_0)) * (freq_data_sample_1 - freq_data_sample_0);

    // Convert the interpolated value back to int if needed
    int result = static_cast<int>(interpolatedValue);

    return result;
}

double ProstateRobotActiveSteeringMode::ConvertMotorTicksPerSecToRpm(Motor *motor)
{
    return motor->GetEncoderVelocity() * (60 / (motor->_ticksPerUnit * 2 * M_PI)); // Get RPM velocity ;
}

//======================== LEGACY VELOCITY CALCULATION CODE ========================
// double unsigned_w_hat = abs(w_hat);
// double a, b, c, d;
// a = 5.921e+07;
// b = -0.08458;
// c = 1.14e+09;
// d = -9.731e-05;
// double desired_vel = unsigned_w_hat * max_rotation_speed_rpm;
// // int des_vel_hex = LinearInterpolation(desired_vel, );
// // std::cout << "Desired Vel: " << desired_vel << std::endl;
// int des_vel_hex = a * exp(b * desired_vel) + c * exp(d * desired_vel);
// // std::cout << "Desired Freq: " << freq << std::endl;
// // double conversion_factor{22009.157};
// // int hex = int(freq * conversion_factor); // 0x432AA764; //
// // std::cout << "Desired in Hex: " << hex << std::endl;
// return des_vel_hex;