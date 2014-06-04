#include "SailingRobot.h"
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <fstream>
#include <cstring>

SailingRobot::SailingRobot() {
/*	wiringPiSetup();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);*/
}

SailingRobot::~SailingRobot() {
/*	digitalWrite(6, LOW);
	std::cout << "sr destruct\n";*/
}


void SailingRobot::init() {

	std::string val, val2, val3, val4;

	try {
		//dbhandler
		m_dbHandler.openDatabase("/root/sailingrobot/asr.db");
	std::cout << "dbh inited\n";

		//servos
		std::string port = m_dbHandler.retriveCell("configs", "1", "mc_portname");
		m_maestroController.setPort(port.c_str());
	std::cout << "maestro inited\n";

		m_rudderServo.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "rs_channel");
		m_rudderServo.setChannel(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_limitmin");
		m_rudderServo.setMin(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_limitmax");
		m_rudderServo.setMax(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_speed");
		m_rudderServo.setSpeed(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rs_acceleration");
		m_rudderServo.setAcceleration(atoi(val.c_str()));
	std::cout << "rudderservo inited\n";
		m_sailServo.setController(&m_maestroController);
		val = m_dbHandler.retriveCell("configs", "1", "ss_channel");
		m_sailServo.setChannel(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_limitmin");
		m_sailServo.setMin(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_limitmax");
		m_sailServo.setMax(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_speed");
		m_sailServo.setSpeed(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "ss_acceleration");
		m_sailServo.setAcceleration(atoi(val.c_str()));
	std::cout << "sailservo inited\n";
		m_windSensor.setController(&m_maestroController);
		m_windSensor.setChannel(5);
	std::cout << "windsensor inited\n";


		//windsensor
	/*	try {
			m_windSensorController.loadConfig("CV7", "/dev/ttyAMA0", 4800);
		} catch(const char* exception) {
			cout << exception << endl;
			return;
		}
		//m_windSensorController.mockDirection(180);
	std::cout << "windsens inited\n";*/



		//gps
		std::string port2 = m_dbHandler.retriveCell("configs", "1", "gps_portname");
		std::string port3 = m_dbHandler.retriveCell("configs", "1", "gps_connectionname");
		m_gpsReader.connectToGPS(port2.c_str(), port3.c_str());
	std::cout << "gpsreader inited\n";

		//coursecalc
		val = m_dbHandler.retriveCell("configs", "1", "cc_tackangle");
		m_courseCalc.setTACK_ANGLE(atoi(val.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "cc_sectorangle");
		m_courseCalc.setSECTOR_ANGLE(atoi(val.c_str()));
	std::cout << "coursecalc inited\n";


		val = m_dbHandler.retriveCell("configs", "1", "rc_commandextreme");
		val2 = m_dbHandler.retriveCell("configs", "1", "rc_commandmedium");
		val3 = m_dbHandler.retriveCell("configs", "1", "rc_commandsmall");
		val4 = m_dbHandler.retriveCell("configs", "1", "rc_commandmidships");
		m_rudderCommand.setCommandValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()), atoi(val4.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "rc_anglemedium");
		val2 = m_dbHandler.retriveCell("configs", "1", "rc_anglesmall");
		val3 = m_dbHandler.retriveCell("configs", "1", "rc_anglemidships");
		m_rudderCommand.setAngleValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()));
	std::cout << "ruddercommand inited\n";

		val = m_dbHandler.retriveCell("configs", "1", "sc_commandclosereach");
		val2 = m_dbHandler.retriveCell("configs", "1", "sc_commandbeamreach");
		val3 = m_dbHandler.retriveCell("configs", "1", "sc_commandbroadreach");
		val4 = m_dbHandler.retriveCell("configs", "1", "sc_commandrunning");
		m_sailCommand.setCommandValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()), atoi(val4.c_str()));
		val = m_dbHandler.retriveCell("configs", "1", "sc_anglebeamreach");
		val2 = m_dbHandler.retriveCell("configs", "1", "sc_anglebroadreach");
		val3 = m_dbHandler.retriveCell("configs", "1", "sc_anglerunnning");
		m_sailCommand.setAngleValues(atoi(val.c_str()), atoi(val2.c_str()), atoi(val3.c_str()));
	std::cout << "sailcommand inited\n";


		//waypoints
		val = m_dbHandler.retriveCell("waypoints", "1", "latitude");
		val2 = m_dbHandler.retriveCell("waypoints", "1", "longitude");
		m_waypointList.add(strtod(val.c_str(), NULL), strtod(val2.c_str(), NULL));
	std::cout << "wp inited\n";

	} catch (const char * error) {
		logError(error);
		exit(1);
	}
}


void SailingRobot::run() {
	
	while(true) {
		//check sensors
		readGPS();
		while (isnan(m_gpsReader.getLatitude())) {
			readGPS();
		}
//		try {
//			m_windSensorController.refreshData();
//		} catch(const char* exception) {
//			cout << exception << endl;
//		}
std::cout << "gpsread\n";
		//do coursecalc
		m_courseCalc.setTWD(m_windSensor.getDirection());
std::cout << "windsensor done\n";
		if (m_courseCalc.getDTW() < 15) {
			m_waypointList.next();
		}

		m_courseCalc.calculateBTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
			m_waypointList.getLatitude(), m_waypointList.getLongitude());
		m_courseCalc.calculateDTW(m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
			m_waypointList.getLatitude(), m_waypointList.getLongitude());

		m_courseCalc.calculateCTS();

std::cout << "cts calulated\n";
		//rudder adjust
        int rudderCommand = m_rudderCommand.getCommand(m_courseCalc.getCTS(), m_gpsReader.getHeading());
		m_rudderServo.setPosition(rudderCommand);


		//sail adjust
        int sailCommand = m_sailCommand.getCommand(m_windSensor.getDirection());
		m_sailServo.setPosition(sailCommand);

std::cout << "servos done\n";
		try {
			m_dbHandler.insertDataLog(sailCommand, rudderCommand, m_courseCalc.getDTW(), m_courseCalc.getBTW(),
			m_courseCalc.getCTS(), m_courseCalc.getTACK(),
			0, "WW",
			m_windSensor.getDirection(), 0, 
			0, m_rudderServo.getPosition(), m_sailServo.getPosition(),
			m_gpsReader.getTimestamp(), m_gpsReader.getLatitude(), m_gpsReader.getLongitude(),
			m_gpsReader.getAltitude(), m_gpsReader.getSpeed(), m_gpsReader.getHeading(),
			m_gpsReader.getMode(), m_gpsReader.getSatellitesUsed());
		} catch (const char * error) {
			logError(error);
		}

		sleep(1);
		m_rudderServo.setPosition(m_rudderCommand.getMidShipsCommand());
		sleep(1);
	}

}


void SailingRobot::shutdown() {
	m_dbHandler.closeDatabase();
}


void SailingRobot::logError(string error) {
	try {
		m_dbHandler.insertErrorLog(error);
	} catch (const char * logError) {
		std::ofstream errorFile;
			errorFile.open("/root/sailingrobot/errors.log", ios::app);
			errorFile << "log error: " << logError << "\n";
			errorFile << "when logging error: " << error << "\n";
		errorFile.close();
	}
}

void SailingRobot::readGPS() {
		try {
			m_gpsReader.readGPS(50000000);
		} catch (const char * error) {
			logError(error);
		}
}