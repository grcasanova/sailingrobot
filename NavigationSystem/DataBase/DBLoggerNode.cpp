#include "DBLoggerNode.h"

#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"
#include "Messages/CourseDataMsg.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "Messages/MarineSensorDataMsg.h"
#include "Messages/CourseDataMsg.h"
#include "SystemServices/Timer.h"
#include "SystemServices/SysClock.h"

#define STATE_INITIAL_SLEEP 100


DBLoggerNode::DBLoggerNode(MessageBus& msgBus, DBHandler& db,int queueSize)
:   ActiveNode(NodeID::DBLoggerNode, msgBus),
    m_db(db),
    m_dbLogger(queueSize, db),
    m_loopTime(0.5),
    m_queueSize(queueSize)

{
    msgBus.registerNode(*this, MessageType::CompassData);
    msgBus.registerNode(*this, MessageType::GPSData);
    msgBus.registerNode(*this, MessageType::WindData);
    // msgBus.registerNode(*this, MessageType::ActuatorPosition);
    msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);
    msgBus.registerNode(*this, MessageType::MarineSensorData);
    
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindState);

    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::CourseData);
    msgBus.registerNode(*this, MessageType::LocalNavigation);
    
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
    
    
}

void DBLoggerNode::processMessage(const Message* msg) {

    std::lock_guard<std::mutex> lock(m_lock);

    MessageType type = msg->messageType();

    switch(type)
    {
        case MessageType::ASPireActuatorFeedback:
        {
            const ASPireActuatorFeedbackMsg* aspMsg = static_cast<const ASPireActuatorFeedbackMsg*>(msg);
            item.m_rudderPosition = aspMsg->rudderFeedback();
            item.m_wingsailPosition = aspMsg->wingsailFeedback();
            item.m_windVaneAngle = aspMsg->windvaneSelfSteeringAngle();
            item.m_radioControllerOn = aspMsg->radioControllerOn();
        }
        break;

        case MessageType::CompassData:
        {
            const CompassDataMsg* compassDataMsg = static_cast<const CompassDataMsg*>(msg);
            item.m_compassHeading = compassDataMsg->heading();
            item.m_compassPitch = compassDataMsg->pitch();
            item.m_compassRoll = compassDataMsg->roll();
        }
        break;

        case MessageType::LocalNavigation:
        {
            const LocalNavigationMsg* localNavigationMsg = static_cast<const LocalNavigationMsg*>(msg);
            item.m_courseToSteer = localNavigationMsg->targetCourse();
            item.m_tack = localNavigationMsg->beatingMode();
            item.m_goingStarboard = localNavigationMsg->targetTackStarboard();
        }
        break;

        case MessageType::GPSData:
        {
            const GPSDataMsg* GPSdataMsg = static_cast<const GPSDataMsg*>(msg);
            item.m_gpsHasFix = GPSdataMsg->hasFix();
            item.m_gpsOnline = GPSdataMsg->gpsOnline();
            item.m_gpsLat = GPSdataMsg->latitude();
            item.m_gpsLon = GPSdataMsg->longitude();
            item.m_gpsUnixTime = GPSdataMsg->unixTime();
            item.m_gpsSpeed = GPSdataMsg->speed();
            item.m_gpsCourse = GPSdataMsg->course();
            item.m_gpsSatellite = GPSdataMsg->satelliteCount();
        }
        break;

        case MessageType::MarineSensorData:
        {
            const MarineSensorDataMsg* marineSensorMsg = static_cast<const MarineSensorDataMsg*>(msg);
            item.m_temperature = marineSensorMsg->temperature();
            item.m_conductivity = marineSensorMsg->conductivity();
            item.m_ph = marineSensorMsg->ph();
        }
        break;

        // case MessageType::WaypointData:
        // {
        //     WaypointDataMsg* waypMsg = (WaypointDataMsg*)msg;
        //     item.m_waypointId = waypMsg->nextId();
        // }

        case MessageType::StateMessage:
        {
            const StateMessage* stateMsg = static_cast<const StateMessage*>(msg);
            item.m_vesselHeading = stateMsg->heading();
            item.m_vesselLat = stateMsg->latitude();
            item.m_vesselLon = stateMsg->longitude();
            item.m_vesselSpeed = stateMsg->speed();
            item.m_vesselCourse = stateMsg->course();
        }
        break;

        case MessageType::WindState:
        {
            const WindStateMsg* windStateMsg = static_cast<const WindStateMsg*>(msg);
            item.m_trueWindSpeed = windStateMsg->trueWindSpeed();
            item.m_trueWindDir = windStateMsg->trueWindDirection();
            item.m_apparentWindSpeed = windStateMsg->apparentWindSpeed();
            item.m_apparentWindDir = windStateMsg->apparentWindDirection();
        }
        break;

        case MessageType::WindData:
        {
            const WindDataMsg* windDataMsg = static_cast<const WindDataMsg*>(msg);
            item.m_windDir = windDataMsg->windDirection();
            item.m_windSpeed = windDataMsg->windSpeed();
            item.m_windTemp = windDataMsg->windTemp();
        }
        break;

        case MessageType::CourseData:
        {
            const CourseDataMsg* courseDataMsg = static_cast<const CourseDataMsg*>(msg);
            item.m_distanceToWaypoint = courseDataMsg->distanceToWP();
            item.m_bearingToWaypoint = courseDataMsg->courseToWP();
        }
        break;

        case MessageType::ServerConfigsReceived:
            updateConfigsFromDB();
        break;

        default:
        return;
    }
}

void DBLoggerNode::start() {
    m_Running.store(true);
    runThread(DBLoggerNodeThreadFunc);
}

void DBLoggerNode::stop() {
    m_Running.store(false);
    stopThread(this);
}


bool DBLoggerNode::init() {
    updateConfigsFromDB();
    return true;
}

void DBLoggerNode::updateConfigsFromDB()
{
    m_loopTime = m_db.retrieveCellAsDouble("config_dblogger","1","loop_time");
}

void DBLoggerNode::DBLoggerNodeThreadFunc(ActiveNode* nodePtr) {

    DBLoggerNode* node = dynamic_cast<DBLoggerNode*> (nodePtr);
    std::string timestamp_str;
    Timer timer;
    Timer timer2;
    timer.start();
    timer2.start();
    node->m_dbLogger.startWorkerThread();

    while(node->m_Running.load() == true) {

        timestamp_str = SysClock::timeStampStr();
        timestamp_str+= ".";
        timestamp_str+= std::to_string(SysClock::millis());

        node->item.m_timestamp_str = timestamp_str;
        node->m_lock.lock();
        node->m_dbLogger.log(node->item);
        node->m_lock.unlock();
        timer.sleepUntil(node->m_loopTime);
        timer.reset();

    }
}
