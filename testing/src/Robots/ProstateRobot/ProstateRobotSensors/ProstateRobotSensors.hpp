#ifndef _ProstateRobotSensors_HPP_
#define _ProstateRobotSensors_HPP_

#include "Timer.hpp"
#include "Logger.hpp"
#include "Packets.hpp"
#include "FPGA_Utilities.hpp"
#include "ForceSensor.hpp"
#include "ProstateRobotEnumerations.hpp"

class ProstateRobotSensors
{
public:
    ProstateRobotSensors(Packets *, FPGA_Utilities *);
    ForceSensor *GetSensor(ProstateRobotAnalogDigitalId);
    void UpdateSensorReadings();
    
    ProstateRobotMotorNames MOTOR;
    // Outgoing and Incoming Packets via SPI
    Packets *_packets;
    // The FPGA Utility object was included to allow for control over LEDs
    FPGA_Utilities *_fpga_util;
    // Prostate Sensors are defined below
    ForceSensor_Config InputForceConfig, NeedleForceConfig;
    ForceSensor _inputForce, _needleForce;
    std::map<ProstateRobotAnalogDigitalId, ForceSensor *> sensor_list;
};

#endif // _ProstateRobotSensors_HPP_