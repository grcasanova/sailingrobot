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

void CANCurrentSensorNode::CANCurrentSensorNode() {
    m_LoopTime = m_db.retrieveCellAsDouble("config_current_sensor","1","loop_time");
}

void CANCurrentSensorNode::processMessage(const Message* message) {

  std::lock_guard<std::mutex> lock(m_lock);

  if(message->messageType() == MessageType::DataRequest)
  {
    // On system startup we won't have any valid data, so don't send any
    if( m_current != DATA_OUT_OF_RANGE ||  m_voltage != DATA_OUT_OF_RANGE || m_element != UNDEFINED )
    {
      MessagePtr CurrentData = std::make_unique<CurrentDataMsg>(message->sourceID(), this->nodeID(), m_current, m_voltage, m_element);
      m_MsgBus.sendMessage(std::move(CurrentData));
    }
  }
  else if(message->messageType() == MessageType::ServerConfigsReceived)
  {
        updateConfigsFromDB();
  }
}

void CANCurrentSensorNode::start()
{
    runThread(CANCurrentSensorNodeThreadFunc);
}

void CANCurrentSensorNode::CANCurrentSensorNodeThreadFunc(ActiveNode* nodePtr) {

  CANCurrentSensorNode* node = dynamic_cast<CANCurrentSensorNode*> (nodePtr);
  Timer timer;
  timer.start();

  while(true)
	{

    node->m_lock.lock();

    /** @todo some current sensors may not deliver voltage data, check it before removing the comment */
    if( not (node->m_current == DATA_OUT_OF_RANGE && /* node->m_voltage == DATA_OUT_OF_RANGE &&*/ node->m_element == UNDEFINED) )
    {
        MessagePtr currentData = std::make_unique<CurrentDataMsg>(node->m_current, node->m_voltage, node->m_element);
        node->m_MsgBus.sendMessage(std::move(CurrentData));
    }
    node->m_lock.unlock();

    timer.sleepUntil(node->m_LoopTime);
    timer.reset();
  }
}
