

#include "Hardwares/Adafruit_INA219.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"


int main (){
	Timer timer;
	timer.start();
	Logger::init("CSlog.log");
	Adafruit_INA219 currentSensor;
	currentSensor.begin();

	while (1){
	
		Logger::info ("Current: " + std::to_string(currentSensor.getCurrent_mA()));
		Logger::info ("Bus voltage: " + std::to_string(currentSensor.getBusVoltage_V()));
		Logger::info ("Shunt voltage: " + std::to_string(currentSensor.getShuntVoltage_mV()));
		timer.sleepUntil(1);
		timer.reset ();
	}
	
}