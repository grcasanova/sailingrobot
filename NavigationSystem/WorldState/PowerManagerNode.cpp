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
  }
  
  void PowerManagerNode::start() {
    runThread(PowerManagerNodeThreadFunc);
  }
  
  void PowerManagerNode::checkCurrentLimits() {
      bool err = 0;
      
      switch(m_element)
        {
            case SAILDRIVE:
                if(m_current > MAX_CURRENT_SAILDRIVE)
                    err = 1
                break;
                
            case WINDVANE_SWITCH:
                if(m_current > MAX_CURRENT_WINDWANE_SWITCH)
                    err = 1;
                break;
                
            case WINDVANE_ANGLE:
                if(m_current > MAX_CURRENT_WINDVANE_ANGLE)
                    err = 1;
                break;
                
            case ACTUATOR_UNIT:
                if(m_current > MAX_CURRENT_ACTUATOR_UNIT)
                    err = 1;
                break;
                
            default:
                break;
        }
        
        /** @todo disable load mosfet? */
        if(err)
            Logger::error("current burn-out detected on the element %s", m_element_str.c_str());
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
