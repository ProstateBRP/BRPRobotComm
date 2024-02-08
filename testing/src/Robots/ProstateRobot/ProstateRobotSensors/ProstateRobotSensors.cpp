#include "ProstateRobotSensors.hpp"

ProstateRobotSensors::ProstateRobotSensors()
{}

// ProstateRobotSensors::ProstateRobotSensors(Packets *packets, FPGA_Utilities *fpga_util) : _packets(packets), _fpga_util(fpga_util)
// {
// 	//=================== Prostate Robot Sensors =====================
// 	// Configuration of sensors
// 	InputForceConfig = {force_sensor_type::ttforce, 0.902477907278600, -1.600328423214212e3, 1970, static_cast<int>(ProstateRobotAnalogDigitalId::INPUT_SENSOR)};
// 	NeedleForceConfig = {force_sensor_type::ttforce, -0.902477907278600, 1.600328423214212e3, 1780, static_cast<int>(ProstateRobotAnalogDigitalId::NEEDLE_SENSOR)};

// 	// These are the sensors
// 	_inputForce = ForceSensor(packets, static_cast<int>(ProstateRobotForceSensor::FORCE_SENSOR), InputForceConfig);
// 	_needleForce = ForceSensor(packets, static_cast<int>(ProstateRobotForceSensor::FORCE_SENSOR), NeedleForceConfig);
// 	// Initialize sensor list
// 	sensor_list =
// 		{
// 			{ProstateRobotAnalogDigitalId::INPUT_SENSOR, &_inputForce},
// 			{ProstateRobotAnalogDigitalId::NEEDLE_SENSOR, &_needleForce},
// 		};
// }

// ForceSensor *ProstateRobotSensors::GetSensor(ProstateRobotAnalogDigitalId cardID)
// {
// 	if (sensor_list.find(cardID) != sensor_list.end())
// 	{
// 		// if the element is found before the end of the map
// 		return sensor_list[cardID];
// 	}
// 	else
// 	{
// 		return NULL;
// 	}
// }

void ProstateRobotSensors::UpdateSensorReadings()
{
	// _inputForce.SetRawForce();
	// _needleForce.SetRawForce();
}