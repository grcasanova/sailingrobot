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
    m_element = UNDEFINED;

  msgBus.registerNode(*this, MessageType::DataRequest);
  msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
  updateConfigsFromDB();
}

CANCurrentSensorNode::~CANCurrentSensorNode(){}

void CANCurrentSensorNode::updateConfigsFromDB() {
    /** @todo To be implemented */
   // m_LoopTime = m_db.retrieveCellAsDouble("config_current_sensor","1","loop_time");
}

void CANCurrentSensorNode::processMessage(const Message* message) {
  if(message->messageType() == MessageType::ServerConfigsReceived)
  {
        updateConfigsFromDB();
  }
}

void CANCurrentSensorNode::processFrame (CanMsg& msg) {
    
    Logger::info("new CAN message received");
    
    if (msg.id == 705) {
        
        // Parse voltage
        m_voltage = (msg.data[1] << 8 | msg.data[0]);
        Logger::info("voltage %d", m_voltage); 
        
        // Parse current
        m_current = (msg.data[3] << 8 | msg.data[2]);
        Logger::info("current %d", m_current); 
        
        // Get sensed part
        m_element = (SensedElement) msg.data[4];
        Logger::info("element %d", m_element); 
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
        MessagePtr currentData = std::make_unique<CurrentDataMsg>(node->m_current, node->m_voltage, node->m_element);
        node->m_MsgBus.sendMessage(std::move(currentData));
    }
    
    node->m_lock.unlock();

    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }
}
