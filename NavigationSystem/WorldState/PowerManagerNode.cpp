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

/** @todo tune-up values */
enum MaxCurrent : int { MAX_CURRENT_NAVIGATION_UNIT = 1000, MAX_CURRENT_ACTUATOR_UNIT, MAX_CURRENT_WINDVANE_ACTUATOR, MAX_CURRENT_SAILDRIVE_ACTUATOR, MAX_CURRENT_RUDDER_ACTUATOR };

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
        case NAVIGATION_UNIT:
            char* elem = "navigation unit";
            break;
            
        case ACTUATOR_UNIT:
            char* elem = "actuator unit";
            break;
            
        case WINDVANE_ACTUATOR:
            char* elem = "windvane actuator";
            break;
            
        case SAILDRIVE_ACTUATOR:
            char* elem = "saildrive actuator";
            break;
            
        case RUDDER_ACTUATOR:
            char* elem = "rudder actuator";
            break;
        
        default:
            char* elem = "undefined";
            break;
    }
    
    Logger::info("current %lf (mA) and voltage %lf (mV) for the element %s", m_current, m_voltage, elem);
#else
    
#endif
  }
  
  void PowerManagerNode::start() {
    runThread(PowerManagerNodeThreadFunc);
  }
  
  void PowerManagerNode::doStuff() {
      switch(m_element)
        {
            case NAVIGATION_UNIT:
                if(m_current > MAX_CURRENT_NAVIGATION_UNIT)
                    throw std::exception("burn-out detected in the NAVIGATION UNIT!");
                break;
                
            case ACTUATOR_UNIT:
                if(m_current > MAX_CURRENT_ACTUATOR_UNIT)
                    throw std::exception("burn-out detected in the ACTUATOR UNIT!");
                break;
                
            case WINDVANE_ACTUATOR:
                if(m_current > MAX_CURRENT_WINDVANE_ACTUATOR)
                    throw std::exception("burn-out detected in the WINDVANE ACTUATOR!");
                break;
                
            case SAILDRIVE_ACTUATOR:
                if(m_current > MAX_CURRENT_SAILDRIVE_ACTUATOR)
                    throw std::exception("burn-out detected in the SAILDRIVE ACTUATOR!");
                break;
                
            case RUDDER_ACTUATOR:
                if(m_current > MAX_CURRENT_RUDDER_ACTUATOR)
                    throw std::exception("burn-out detected in the RUDDER ACTUATOR!");
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
      node->doStuff();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
