#ifndef _ProstateRobotModeBase_HPP_
#define _ProstateRobotModeBase_HPP_

#include "ProstateRobotConstants.hpp"
#include "ProstateRobotMotionController.hpp"
#include <mutex>

class ProstateRobotModeBase
{
public:
    ProstateRobotModeBase(ProstateRobotMotionController *);
    
    ProstateRobotConstants kConstants;
    ProstateRobotModes kRobotModes;
    ProstateRobotMotionController *motion_ctrl;
    
    bool IsFootPedalPressed();
    void RunIdle();
    virtual void Run(const string &current_state = "");
    void SetName(string name){name_ = name;}
    string GetName(){return name_;}

private:
    string name_;
};

#endif // _ProstateRobotModeBase_HPP_