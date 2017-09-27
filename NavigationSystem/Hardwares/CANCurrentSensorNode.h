/****************************************************************************************
 *
 * File:
 * 		CANCurrentSensorNode.h
 *
 * Purpose:
 *		Process messages from the CAN-Service
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#pragma once


#include <mutex>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

#include "DataBase/DBHandler.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/CurrentDataMsg.h"
#include "SystemServices/Timer.h"
#include "SystemServices/Logger.h"

const int DATA_OUT_OF_RANGE = -2000;


class CANCurrentSensorNode : public ActiveNode, public CANFrameReceiver {
public:

    CANCurrentSensorNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& canService);
    ~CANCurrentSensorNode();

	/* data */
    void getData();

	///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

	///----------------------------------------------------------------------------------
 	/// Attempts to connect to the current sensor.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init() {return true;};
    
    virtual void processFrame (CanMsg& msg) override;
    
	virtual void processMessage(const Message* message);

	void start();

private:

	static void CANCurrentSensorNodeThreadFunc(ActiveNode* nodePtr);

	float m_current;			// in mA
	float m_voltage;			// in mV
	SensedElement m_element;    // the element measured
	double m_LoopTime;			// in seconds
	DBHandler& m_db;

	std::mutex m_lock;
};
