#ifndef __ProstateRobotManualMode_HPP_
#define __ProstateRobotManualMode_HPP_

#include "ProstateRobotModeBase.hpp"

class ProstateRobotManualMode : public ProstateRobotModeBase
{
public:
    ProstateRobotManualMode(ProstateRobotMotionController *);
    virtual void Run(const string &current_state = "");
    // double ConvertMotorTicksPerSecToRpm(Motor *);
    int LinearInterpolation(double des_rpm, const vector<double> &, const vector<int> &);
    void ClosedLoopControl();

private:
    double des_vel_ = 11; // rpm
    Timer timer;
    vector<int> frequency_positive_dir;
    vector<int> frequency_negative_dir;
    vector<double> rpm_positive_dir;
    vector<double> rpm_negative_dir;
};

#endif //__ProstateRobotManualMode_HPP_