/****************************************************************************************
*
* File:
* 		SailControlNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the SailControlNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "LowLevelControllers/SailControlNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/ActuatorPositionMsg.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/WindDataMsg.h"
#include "Messages/StateMessage.h"
#include "Math/Utility.h"


// For std::this_thread
#include <chrono>
#include <thread>
//#include "All.h"
#include <math.h>

#define SAIL_CONTROLNODE_TEST_COUNT   5


class SailControlNodeSuite : public CxxTest::TestSuite
{
public:

  SailControlNode* sControlNode;
  DBHandler* dbHandler;
  MockNode* mockNode;
  bool nodeRegistered = false;

  double loopTime = 0.5;
  double MaxSailAngle = 90;
  double MinSailAngle = 15;

  std::thread* thr;
  int testCount = 0;
  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  // ----------------
  // Send messages
  // ----------------
  static void runMessageLoop()
  {
    msgBus().run();
  }

  // ----------------
  // Setup the objects to test
  // ----------------
  void setUp()
  {
    // Test Node for message
    mockNode = new MockNode(msgBus(), nodeRegistered);
    // setup them up once in this test, delete them when the program closes
    if(sControlNode == 0)
    {
        dbHandler = new DBHandler("../test_asr.db");
        Logger::DisableLogging();


        sControlNode = new SailControlNode(msgBus(), *dbHandler, .5, MaxSailAngle, MinSailAngle, 90, 0, 0);
        sControlNode->start();

        std::this_thread::sleep_for(std::chrono::milliseconds(2600));
        thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  // ----------------
  // End of test when all test have been successfull
  // ----------------
  void tearDown()
  {
    if(testCount == SAIL_CONTROLNODE_TEST_COUNT)
    {
       std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      delete sControlNode;
      delete dbHandler;
      delete mockNode;
      //std::cout << std::endl << " #### DELETE OBJECT of the test #### " << std::endl;
      // Stay here for process the last message which return system::error
      //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      thr->detach();
      delete thr;
    }
    else {
            delete mockNode;
    }
  }

  // ----------------
  // Test Initialisation of the object
  // ----------------
  void test_SailControlNodeInit(){
    TS_ASSERT(nodeRegistered);
  }

  // ----------------
  // Test to see if a message concerning the node will be listened and process
  // ----------------
  void test_SailControlMessageListener(){
    double appWindDirection = 45;
    double appWindSpeed = 10;
    double appWindTemp = 15;

    MessagePtr windData =  std::make_unique<WindDataMsg>(appWindDirection,appWindSpeed,appWindTemp);
    msgBus().sendMessage(std::move(windData));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    TS_ASSERT(mockNode->m_MessageReceived);
    TS_ASSERT_EQUALS(mockNode->m_WindDir,appWindDirection);
    TS_ASSERT_EQUALS(mockNode->m_WindSpeed,appWindSpeed);
    TS_ASSERT_EQUALS(mockNode->m_WindTemp,appWindTemp);

    //Check if there is the same result by the processing next to the Compass data has been received
    int sailAngle = -Utility::sgn(appWindDirection)*(((MinSailAngle-MaxSailAngle)*std::abs(appWindDirection)/180)+MaxSailAngle);
    double sailControlNodeSailAngle = mockNode->m_sailPosition;
    TS_ASSERT_EQUALS(sailControlNodeSailAngle,sailAngle);
  }

  // ----------------
  // Test to see if a message concerning the node will be listened and process
  // ----------------
  void test_SailControlNegativeBeamingDirection(){
    double appWindDirection = -90;
    double appWindSpeed = 10;
    double appWindTemp = 15;

    MessagePtr windData =  std::make_unique<WindDataMsg>(appWindDirection,appWindSpeed,appWindTemp);
    msgBus().sendMessage(std::move(windData));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    TS_ASSERT(mockNode->m_MessageReceived);
    TS_ASSERT_EQUALS(mockNode->m_WindDir,appWindDirection);
    TS_ASSERT_EQUALS(mockNode->m_WindSpeed,appWindSpeed);
    TS_ASSERT_EQUALS(mockNode->m_WindTemp,appWindTemp);

    //Check if there is the same result by the processing next to the Compass data has been received
    int sailAngle = -Utility::sgn(appWindDirection)*(((MinSailAngle-MaxSailAngle)*std::abs(appWindDirection)/180)+MaxSailAngle);
    double sailControlNodeSailAngle = mockNode->m_sailPosition;
    TS_ASSERT_EQUALS(sailControlNodeSailAngle,sailAngle);
  }

  // ----------------
  // Test to see if a message concerning the node will be listened and process
  // ----------------
  void test_SailControlRunningConfig(){
    double appWindDirection = 180;
    double appWindSpeed = 10;
    double appWindTemp = 15;

    MessagePtr windData =  std::make_unique<WindDataMsg>(appWindDirection,appWindSpeed,appWindTemp);
    msgBus().sendMessage(std::move(windData));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    TS_ASSERT(mockNode->m_MessageReceived);
    TS_ASSERT_EQUALS(mockNode->m_WindDir,appWindDirection);
    TS_ASSERT_EQUALS(mockNode->m_WindSpeed,appWindSpeed);
    TS_ASSERT_EQUALS(mockNode->m_WindTemp,appWindTemp);

    //Check if there is the same result by the processing next to the Compass data has been received
    int sailAngle = -Utility::sgn(appWindDirection)*(((MinSailAngle-MaxSailAngle)*std::abs(appWindDirection)/180)+MaxSailAngle);
    double sailControlNodeSailAngle = mockNode->m_sailPosition;
    TS_ASSERT_EQUALS(sailControlNodeSailAngle,sailAngle);
  }

};
