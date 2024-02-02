#ifndef _ProstateRobotAutomatedHomingMode_HPP_
#define _ProstateRobotAutomatedHomingMode_HPP_

#include "ProstateRobotModeBase.hpp"

class ProstateRobotAutomatedHomingMode : public ProstateRobotModeBase
{
public:
    ProstateRobotAutomatedHomingMode(ProstateRobotMotionController *);
    void SetMotorSetPointsToHome();
    virtual void Run(const string &);
};

#endif //_ProstateRobotAutomatedHomingMode_HPP_