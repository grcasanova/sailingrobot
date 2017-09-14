/****************************************************************************************
 *
 * File:
 * 		PowerManagerNode.h
 *
 * Purpose:
 *		Process messages from the voltage and current sensors
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#pragma once

#include "Messages/CurrentDataMsg.h"
#include "SystemServices/Timer.h"
#include "MessageBus/Message.h"
#include "DataBase/DBHandler.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"

#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

class PowerManagerNode : public ActiveNode {
public:

  PowerManagerNode(MessageBus& msgBus, DBHandler& dbhandler);
  ~PowerManagerNode();

  bool init();

  /*
  * Processes the message received
  */
  void processMessage(const Message* msg);

  /*
  * Starts the worker thread
  */
  void start();

private:

  /*
  * Update values from the database as the loop time of the thread and others parameters
  */
  void updateConfigsFromDB();

  /*
  * For future use
  */
  void doStuff();

  /*
  * Gets to process the message if the message received comes from the current sensor
  */
  void processSensorMessage(CurrentSensorData* msg);

  /*
  * The function that thread works on
  */
  static void PowerManagerNodeThreadFunc(ActiveNode* nodePtr);

  /*
  * Private variables
  */
  float m_current;
  float m_sensor;
  SensedElement m_element;

  double m_LoopTime;
  std::mutex m_lock;
  DBHandler& m_db;
};
