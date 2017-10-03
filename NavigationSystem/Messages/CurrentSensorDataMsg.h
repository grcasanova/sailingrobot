/****************************************************************************************
 *
 * File:
 * 		CurrentDataMsg.h
 *
 * Purpose:
 *		A CurrentDataMsg contains information about the sensed current (and voltage)
 *      of a certain element (whole unit or single actuator)
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"

enum SensedElement : uint8_t { SAILDRIVE, WINDVANE_SWITCH, WINDVANE_ANGLE, ACTUATOR_UNIT, UNDEFINED };

class CurrentSensorDataMsg : public Message {
public:
	CurrentSensorDataMsg(NodeID destinationID, NodeID sourceID, uint16_t current, uint16_t voltage, SensedElement element)
		:Message(MessageType::CurrentSensorData, sourceID, destinationID), m_current(current), m_voltage(voltage), m_element(element) { }

	CurrentSensorDataMsg(uint16_t current, uint16_t voltage, SensedElement element)
		:Message(MessageType::CurrentSensorData, NodeID::None, NodeID::None), m_current(current), m_voltage(voltage), m_element(element)
	{

	}

	CurrentSensorDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
        uint8_t element = 0;
		if(	!deserialiser.readInt(m_current) ||
			!deserialiser.readInt(m_voltage) ||
			!deserialiser.readInt(element)
         )
		{
			m_valid = false;
		}
 		m_element = (SensedElement) element;
	}

	virtual ~CurrentSensorDataMsg() { }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);
        
		serialiser.serialise(m_current);
		serialiser.serialise(m_voltage);
		serialiser.serialise((uint8_t)m_element);
	}

	uint16_t getCurrent() const { return m_current; }
	uint16_t getVoltage() const { return m_voltage; }
	SensedElement getSensedElement() const { return m_element; }

private:
	uint16_t m_current;			// in mA
	uint16_t m_voltage;			// in mV
	SensedElement m_element;    // the element measured
};
