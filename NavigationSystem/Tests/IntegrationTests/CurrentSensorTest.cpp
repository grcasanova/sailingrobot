/****************************************************************************************
*
* File:
* 		CANCurrentSensorTest.cpp
*
* Purpose:
*		Integration test for the Current Sensors, tests if we can receive and read the messages from the sensors
*
*
* Developer Notes:
*
*
***************************************************************************************/

#include "DataBase/DBHandler.h"
#include "Hardwares/CANCurrentSensorNode.h"
#include "Messages/CurrentDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/PowerManagerNode.h"

CANService canService;
DBHandler dbHandler("../asr.db");
MessageBus msgBus;
CANCurrentSensorNode* sensorCanNode;
PowerManagerNode* sensorProc;

void messageLoop() {
    msgBus.run();
}

int main() {
  auto future = canService.start();

  sensorCanNode = new CANCurrentSensorNode(msgBus, dbHandler, canService);
  sensorCanNode->start();

  sensorProc = new PowerManagerNode(msgBus, dbHandler);
  sensorProc->start();

  std::thread thr(messageLoop);
  thr.detach();
  int now;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
}
