/****************************************************************************************
*
* File:
* 		PowerManagerNode.cpp
*
* Purpose:
*		 Process messages from the CANCurrentSensor to log power consumption data
*
* Developer Notes:
*
*
***************************************************************************************/

#include "PowerManagerNode.h"
#include <inttypes.h>

/** @todo tune-up values and set the enum as a class */
enum MaxCurrent { MAX_CURRENT_SAILDRIVE = 1000, MAX_CURRENT_WINDWANE_SWITCH, MAX_CURRENT_WINDVANE_ANGLE, MAX_CURRENT_ACTUATOR_UNIT, MAX_CURRENT_NAVIGATION_UNIT };

PowerManagerNode::PowerManagerNode(MessageBus& msgBus, DBHandler& dbhandler)
  : ActiveNode(NodeID::PowerManagerNode, msgBus), m_LoopTime(0.5), m_db(dbhandler) {
    msgBus.registerNode(*this, MessageType::CurrentSensorData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
    updateConfigsFromDB();
  }

  PowerManagerNode::~PowerManagerNode() {

  }

  void PowerManagerNode::updateConfigsFromDB(){
      m_LoopTime = m_db.retrieveCellAsDouble("config_power_manager","1","loop_time");
  }

  bool PowerManagerNode::init() {
    updateConfigsFromDB();
    return true;
  }

  void PowerManagerNode::processMessage(const Message* msg) {   
    MessageType type = msg->messageType();
    switch (type) {
      case MessageType::CurrentSensorData :
        processSensorMessage((CurrentSensorDataMsg*) msg);
        break;
      case MessageType::ServerConfigsReceived :
        updateConfigsFromDB();
        break;
      default:
        return;
    }
  }

  void PowerManagerNode::processSensorMessage(CurrentSensorDataMsg* msg) {
    m_current = msg->getCurrent();
    m_voltage = msg->getVoltage();
    m_element = msg->getSensedElement();
    m_element_str = msg->getSensedElementStr();
    
    Logger::info("current %" PRIu16 " (mA) and voltage %" PRIu16 " (mV) for the element %s", m_current, m_voltage, m_element_str.c_str());    
  }
  
  void PowerManagerNode::start() {
    runThread(PowerManagerNodeThreadFunc);
  }
  
  /** @todo to be implemented */
  void PowerManagerNode::checkCurrentLimits() {
      switch(m_element)
        {
            case SAILDRIVE:
                if(m_current > MAX_CURRENT_SAILDRIVE)
                    // Do smt
                break;
                
            case WINDVANE_SWITCH:
                if(m_current > MAX_CURRENT_WINDWANE_SWITCH)
                break;
                
            case WINDVANE_ANGLE:
                if(m_current > MAX_CURRENT_WINDVANE_ANGLE)
                break;
                
            case ACTUATOR_UNIT:
                if(m_current > MAX_CURRENT_ACTUATOR_UNIT)
                break;
                
            default:
                break;
        }
  }

  void PowerManagerNode::PowerManagerNodeThreadFunc(ActiveNode* nodePtr) {
    PowerManagerNode* node = dynamic_cast<PowerManagerNode*> (nodePtr);

    Timer timer;
    timer.start();

    while(true) {
      node->m_lock.lock();
      node->checkCurrentLimits();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
