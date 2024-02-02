#ifndef __ProstateRobotFrequencySweepMode_HPP_
#define __ProstateRobotFrequencySweepMode_HPP_

#include "ProstateRobotModeBase.hpp"
#include <chrono>
#include <thread>
#include <algorithm>

class ProstateRobotFrequencySweepMode : public ProstateRobotModeBase
{
public:
    ProstateRobotFrequencySweepMode(ProstateRobotMotionController *);

    void Run(const std::string &);
    void RunInitialization();
    void PopulateTestFrequencies();
    void RunFrequencySweep();
    void PerformCoarseSearch();
    void PerformFineSearch();
    bool isCoarseSearchFinished();
    bool isFineSearchFinished();
    bool isInitializationFinished();

    bool sweep_started_{false};
    bool sweep_finished_{false};
    bool coarse_search_finished_{false};
    bool fine_search_finished_{false};

    const int kFreqInc{4063228};
    const int kRunTimeSec{5}; // Run each freq for 5 sec
    const int kCoarseFreqInc{22009000};
    const int kTestingInstances{24};
    const int kUpperFreqRangeHz{55000}; 
    const int kLowerFreqRangeHz{45000};
    const int kFreqToHexConversionFactor{22009};
    const double kStopVel{0.1}; // RPM

    int current_freq_;
    // int start_freq_{0x45872743};
    int start_freq_;
    Timer timer;
    vector<double> average_vel_vec_;
    vector<double> final_vel_vec_;
    vector<int> test_freq_;
};

#endif //__ProstateRobotFrequencySweepMode_HPP_