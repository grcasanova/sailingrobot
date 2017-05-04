#pragma once

#include "HardwareServices/CAN_Services/N2kMsg.h"
<<<<<<< HEAD
=======
#include "../cxxtest/cxxtest/TestSuite.h"
#include <iostream>
>>>>>>> extend_CAN_Service

class CANMessageSuite : public CxxTest::TestSuite {
    public:

    void setUp() {}
    void tearDown() {}

    void test_CANMessageConversion() {
        N2kMsg Nmsg;
        CanMsg Cmsg;

        Nmsg.PGN = 130311;
        std::vector<uint8_t> data = {1,2,3,4,5,6,7,8};
        Nmsg.Data = data;

        N2kMsgToCanMsg(Nmsg, Cmsg);
        N2kMsg Nmsg2;
        CanMsgToN2kMsg(Cmsg, Nmsg2);

        TS_ASSERT_EQUALS(Nmsg.PGN, Nmsg2.PGN);

        for(int i=0; i<8; i++) {
            TS_ASSERT_EQUALS(Nmsg.Data[i], Nmsg2.Data[i]);
        }
        
        
    }
};
