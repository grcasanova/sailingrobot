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

CANCurrentSensorNode::CANCurrentSensorNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& can_service)
: ActiveNode(NodeID::CurrentSensor, msgBus), m_LoopTime(0.5), m_db(dbhandler)
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
    m_LoopTime = m_db.retrieveCellAsDouble("config_current_sensor","1","loop_time");
}

void CANCurrentSensorNode::processMessage(const Message* message) {
  if(message->messageType() == MessageType::ServerConfigsReceived)
  {
        updateConfigsFromDB();
  }
}

typedef union { float num; unsigned char bytes[4]; } FLOAT;

void CANCurrentSensorNode::processFrame (CanMsg& msg) {
    
    Logger::info("new CAN message");
    
	FLOAT rawData;

	if (msg.id == 705) {
        
        // Parse voltage
        for(int i = 0; i < 4; i++)
            rawData.bytes[i] = msg.data[i];

        m_voltage = rawData.num;
        
       Logger::info("voltage %lf", m_voltage); 
        
        // Parse current
        for(int i = 4; i < 8; i++)
            rawData.bytes[i-4] = msg.data[i];

        m_current = rawData.num;
        
        Logger::info("current %lf", m_current); 

        // Get sensed unit
        m_element = (SensedElement) msg.data[8];
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
