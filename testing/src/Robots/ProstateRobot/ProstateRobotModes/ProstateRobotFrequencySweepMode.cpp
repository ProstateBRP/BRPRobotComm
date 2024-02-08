#include "ProstateRobotFrequencySweepMode.hpp"

ProstateRobotFrequencySweepMode::ProstateRobotFrequencySweepMode(ProstateRobotMotionController *motion_ctrl) : ProstateRobotModeBase(motion_ctrl)
{
    SetName(kRobotModes.FREQUENCY_SWEEP_MODE);
    // Start the search from the manufacturer recommended freq
    start_freq_ = kUpperFreqRangeHz * kFreqToHexConversionFactor;
    current_freq_ = start_freq_;
}

void ProstateRobotFrequencySweepMode::Run(const std::string &current_state)
{
    if (!isInitializationFinished())
    {
        RunInitialization();
    }
    else
    {
        RunFrequencySweep();
    }
}

void ProstateRobotFrequencySweepMode::PopulateTestFrequencies()
{
    // for (int i = 0; i < kTestingInstances; i++)
    // {
    //     test_freq_.push_back(start_freq_ - (kFreqInc * i));
    // }
    // current_freq_ = test_freq_.at(0);
    // Motor *rotation_motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
    // rotation_motor->_velocity = current_freq_;
}

void ProstateRobotFrequencySweepMode::RunInitialization()
{
    if (!isCoarseSearchFinished())
    {
        PerformCoarseSearch();
    }
    else if (!isFineSearchFinished())
    {
        PerformFineSearch();
    }
}

void ProstateRobotFrequencySweepMode::RunFrequencySweep()
{
    // if (sweep_finished_)
    // {
    //     return;
    // }

    // if (!sweep_started_)
    // {
    //     std::cout << "********** Frequency sweep started **********" << std::endl;
    //     timer.tic();
    //     sweep_started_ = true;
    //     std::cout << "Current Freq: " << current_freq_ << std::endl;
    // }

    // timer.toc();
    // Motor *rotation_motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);

    // if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec)
    // {
    //     rotation_motor->MoveMotor();
    //     // Store average velocity in RPM
    //     average_vel_vec_.push_back((rotation_motor->GetEncoderVelocity() / rotation_motor->_ticksPerUnit) / 2 * M_PI);
    // }
    // else
    // {
    //     // Send stop command for one second
    //     if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec + 1)
    //     {
    //         rotation_motor->StopMotor();
    //     }
    //     // Go to next frequency after one second of waiting
    //     else
    //     {
    //         // Find the index of the current tested frequency
    //         auto found_it = std::find(test_freq_.begin(), test_freq_.end(), current_freq_);

    //         // Reached the end of the vector
    //         if (found_it != test_freq_.end())
    //         {
    //             // Have not yet reached the end
    //             int index = found_it - test_freq_.begin();
    //             std::cout << "Test Frequency No: " << index << std::endl;
    //             // Update the final velocity vector for the tested frequency
    //             final_vel_vec_.push_back(std::accumulate(average_vel_vec_.begin(), average_vel_vec_.end(), 0.0) / average_vel_vec_.size());
    //             // Bound check the next frequency index
    //             if (index < test_freq_.size() - 1)
    //             {
    //                 current_freq_ = test_freq_.at(index + 1);
    //                 std::cout << "Current Freq: " << current_freq_ << std::endl;
    //                 rotation_motor->_velocity = current_freq_;
    //                 // reset the timer
    //                 timer.tic();
    //                 // reset the average velocity vector
    //                 average_vel_vec_.clear();
    //             }
    //             // Reached the end of the bound
    //             else
    //             {
    //                 std::cout << "********** Frequency sweep finished **********" << std::endl;
    //                 sweep_finished_ = true;
    //                 // Disable the rotation motor
    //                 rotation_motor->_enabled = false;
    //                 Logger &logger = Logger::GetInstance();
    //                 // Print the results to the terminal
    //                 string msg{"List of measured velocities and the corresponding frequencies:"};
    //                 logger.Log(msg, logger::INFO, true);
    //                 // Sleep for 0.5 second
    //                 std::chrono::milliseconds duration(100);
    //                 std::this_thread::sleep_for(duration);
    //                 for (int i = 0; i < final_vel_vec_.size(); i++)
    //                 {
    //                     // Save the recorded data in the log
    //                     msg.clear();
    //                     msg = "Tested Frequency:  " + to_string(test_freq_.at(i)) + ",   measured velocity:  " + to_string(final_vel_vec_.at(i)) + " (RPM)";
    //                     logger.Log(msg, logger::INFO, true);
    //                     std::this_thread::sleep_for(duration);
    //                 }
    //             }
    //         }
    //     }
    // }
}

// This function looks for a frequency where the motor starts turning
void ProstateRobotFrequencySweepMode::PerformCoarseSearch()
{
    // // If the initialization has been performed
    // Motor *rotation_motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
    // rotation_motor->_velocity = current_freq_;

    // if (!sweep_started_)
    // {
    //     std::cout << "********** Coarse search started **********" << std::endl;
    //     timer.tic();
    //     sweep_started_ = true;
    //     std::cout << "Coarse search: current freq is => " << current_freq_ << std::endl;
    // }
    // timer.toc();
    // if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec)
    // {
    //     rotation_motor->MoveMotor();
    //     // Store average velocity in RPM
    //     average_vel_vec_.push_back((rotation_motor->GetEncoderVelocity() / rotation_motor->_ticksPerUnit) / 2 * M_PI);
    // }
    // else
    // {
    //     // Send stop command for one second
    //     if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec + 1)
    //     {
    //         rotation_motor->StopMotor();
    //     }
    //     // Go to next frequency after one second of waiting
    //     else
    //     {
    //         // Check if motion was detected
    //         double avg_vel = std::accumulate(average_vel_vec_.begin(), average_vel_vec_.end(), 0.0) / average_vel_vec_.size();
    //         // No motion detected
    //         if (avg_vel <= kStopVel)
    //         {
    //             // Decrease the frequency
    //             current_freq_ -= kCoarseFreqInc;
    //             std::cout << "Coarse search: current freq is => " << current_freq_ << std::endl;
    //             timer.tic();
    //         }
    //         // Motion detected
    //         else
    //         {
    //             // Use the tested frequency as the starting point for the fine initialization run
    //             start_freq_ = current_freq_;
    //             // reset sweep start flag for the main sweep loop
    //             sweep_started_ = false;
    //             coarse_search_finished_ = true;
    //             std::cout << "********** Coarse search finished **********" << std::endl;
    //         }
    //         // Clear the average vel vector
    //         average_vel_vec_.clear();
    //     }
    // }
}

void ProstateRobotFrequencySweepMode::PerformFineSearch()
{
    // Motor *rotation_motor = motion_ctrl->motors->GetMotor(ProstateRobotMotor::ROTATION);
    // rotation_motor->_velocity = current_freq_;

    // if (!sweep_started_)
    // {
    //     std::cout << "********** Fine search started **********" << std::endl;
    //     timer.tic();
    //     sweep_started_ = true;
    //     std::cout << "Fine search: current freq is => " << current_freq_ << std::endl;
    // }
    // timer.toc();
    // if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec)
    // {
    //     rotation_motor->MoveMotor();
    //     // Store average velocity in RPM
    //     average_vel_vec_.push_back((rotation_motor->GetEncoderVelocity() / rotation_motor->_ticksPerUnit) / 2 * M_PI);
    // }
    // else
    // {
    //     // Send stop command for one second
    //     if (timer.ConvertMicrosecToSec(timer.time()) < kRunTimeSec + 1)
    //     {
    //         rotation_motor->StopMotor();
    //     }
    //     // Go to next frequency after one second of waiting
    //     else
    //     {
    //         // Check if the average vel is smaller than the threshold
    //         double avg_vel = std::accumulate(average_vel_vec_.begin(), average_vel_vec_.end(), 0.0) / average_vel_vec_.size();
    //         if (avg_vel <= kStopVel)
    //         {
    //             // Use the tested frequency as the starting point for the sweep
    //             start_freq_ = current_freq_;
    //             PopulateTestFrequencies();
    //             // reset sweep start flag for the main sweep loop
    //             sweep_started_ = false;
    //             // Finish the search
    //             fine_search_finished_ = true;
    //             std::cout << "********** Fine search finished **********" << std::endl;
    //         }
    //         // Increase the frequency
    //         else
    //         {
    //             current_freq_ += kFreqInc;
    //             std::cout << "Fine search: current freq is => " << current_freq_ << std::endl;
    //             timer.tic();
    //         }
    //         // Clear the average vel vector
    //         average_vel_vec_.clear();
    //     }
    // }
}

bool ProstateRobotFrequencySweepMode::isCoarseSearchFinished()
{
    return coarse_search_finished_;
}

bool ProstateRobotFrequencySweepMode::isFineSearchFinished()
{
    return fine_search_finished_;
}

bool ProstateRobotFrequencySweepMode::isInitializationFinished()
{
    // If the test freq vector is not yet populated
    return !test_freq_.empty();
}