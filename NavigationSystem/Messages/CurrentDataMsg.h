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

/** @todo refactor according to what is being used */
enum SensedElement : int { NAVIGATION_UNIT = 0, ACTUATOR_UNIT, WINDVANE_ACTUATOR, SAILDRIVE_ACTUATOR, RUDDER_ACTUATOR, UNDEFINED };

class CurrentDataMsg : public Message {
public:
	CurrentDataMsg(NodeID destinationID, NodeID sourceID, float current, float voltage, SensedElement element)
		:Message(MessageType::CurrentData, sourceID, destinationID), m_current(current), m_voltage(voltage), m_element(element) { }

	CurrentDataMsg(float current, float voltage, SensedElement element)
		:Message(MessageType::CourseData, NodeID::None, NodeID::None), m_current(current), m_voltage(voltage), m_element(element)
	{

	}

	CurrentDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readFloat(m_current) ||
			!deserialiser.readFloat(m_voltage) ||
			!deserialiser.readInt(m_element))
		{
			m_valid = false;
		}
	}

	virtual ~CurrentDataMsg() { }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);
		serialiser.serialise(m_current);
		serialiser.serialise(m_voltage);
		serialiser.serialise(m_element);
	}

	float getCurrent() const { return m_current; }
	float getVoltage() const { return m_voltage; }
	float getSensedElement() const { return m_element; }

private:
	float m_current;			// in mA
	float m_voltage;			// in mV
	SensedElement m_element;    // the element measured
};
