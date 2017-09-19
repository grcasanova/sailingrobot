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

/** @todo tune-up values and set the enum as a class */
enum MaxCurrent { MAX_CURRENT_SAILDRIVE = 1000, MAX_CURRENT_WINDWANE_SWITCH, MAX_CURRENT_WINDVANE_ANGLE, MAX_CURRENT_ACTUATOR_UNIT, MAX_CURRENT_NAVIGATION_UNIT };

PowerManagerNode::PowerManagerNode(MessageBus& msgBus, DBHandler& dbhandler)
  : ActiveNode(NodeID::PowerManagerNode, msgBus), m_LoopTime(0.5), m_db(dbhandler) {
    msgBus.registerNode(*this, MessageType::CurrentData);
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
      case MessageType::CurrentData :
        processSensorMessage((CurrentDataMsg*) msg);
        break;
      case MessageType::ServerConfigsReceived :
        updateConfigsFromDB();
        break;
      default:
        return;
    }
  }

  void PowerManagerNode::processSensorMessage(CurrentDataMsg* msg) {
    m_current = msg->getCurrent();
    m_voltage = msg->getVoltage();
    m_element = msg->getSensedElement();

#ifdef DEBUG
    // Print sensed element in a human-friendly way
    switch(m_element)
    {
        case SAILDRIVE:
            char* elem = "saildrive";
            break;
            
        case WINDWANE_SWITCH:
            char* elem = "windvane switch";
            break;
            
        case WINDVANE_ANGLE:
            char* elem = "windvane angle";
            break;
            
        case ACTUATOR_UNIT:
            char* elem = "actuator unit";
            break;
            
        case NAVIGATION_UNIT:
            char* elem = "navigation unit";
            break;
        
        default:
            char* elem = "undefined";
            break;
    }
    
    Logger::info("current %lf (mA) and voltage %lf (mV) for the element %s", m_current, m_voltage, elem);    
#endif
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
                
            case WINDWANE_SWITCH:
                if(m_current > MAX_CURRENT_WINDWANE_SWITCH)
                break;
                
            case WINDVANE_ANGLE:
                if(m_current > MAX_CURRENT_WINDVANE_ANGLE)
                break;
                
            case ACTUATOR_UNIT:
                if(m_current > MAX_CURRENT_ACTUATOR_UNIT)
                break;
                
            case NAVIGATION_UNIT:
                if(m_current > MAX_CURRENT_NAVIGATION_UNIT)
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
