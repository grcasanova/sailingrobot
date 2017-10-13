/****************************************************************************************
*
* File:
* 		CANCurrentSensorNode.cpp
*
* Purpose:
*		 Process messages from the CAN-Service.
*
* Developer Notes:
*		
*
***************************************************************************************/

#include "CANCurrentSensorNode.h"

CANCurrentSensorNode::CANCurrentSensorNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& canService)
: ActiveNode(NodeID::CurrentSensor, msgBus), CANFrameReceiver(canService, {705}), m_LoopTime(0.5), m_db(dbhandler)
{
    m_current = DATA_OUT_OF_RANGE;
    m_voltage = DATA_OUT_OF_RANGE;
    m_element = (SensedElement) UNDEFINED;
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

CANCurrentSensorNode::~CANCurrentSensorNode(){
    
}

void CANCurrentSensorNode::updateConfigsFromDB() 
{
   m_LoopTime = m_db.retrieveCellAsDouble("config_can_current_sensors","1","loop_time");
}

void CANCurrentSensorNode::processMessage(const Message* message) {
  if(message->messageType() == MessageType::ServerConfigsReceived)
  {
        updateConfigsFromDB();
  }
}

void CANCurrentSensorNode::processFrame (CanMsg& msg) {
        
    if (msg.id == 705) {
        
        // Parse voltage
        m_voltage = (msg.data[1] << 8 | msg.data[0]);
        
        // Parse current
        m_current = (msg.data[3] << 8 | msg.data[2]);
        
        // Get sensed part
        m_element = (SensedElement) msg.data[4];
	}
}

void CANCurrentSensorNode::start() {
    runThread(CANCurrentSensorNodeThreadFunc);
}

void CANCurrentSensorNode::CANCurrentSensorNodeThreadFunc(ActiveNode* nodePtr) {

  CANCurrentSensorNode* node = dynamic_cast<CANCurrentSensorNode*> (nodePtr);
  Timer timer;
  timer.start();

  while(true)
  {
      node->m_lock.lock();

      if( not (node->m_current == DATA_OUT_OF_RANGE && node->m_voltage == DATA_OUT_OF_RANGE && node->m_element == UNDEFINED) )
      {
          MessagePtr currentData = std::make_unique<CurrentSensorDataMsg>(node->m_current, node->m_voltage, node->m_element);
          node->m_MsgBus.sendMessage(std::move(currentData));
      }
      
      node->m_lock.unlock();
      
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
  }
}
