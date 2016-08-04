/****************************************************************************************
 *
 * File:
 * 		WindDataMsg.h
 *
 * Purpose:
 *		Contains the following data: Wind temperature, wind direction and wind speed.
 *		Commonly generated by a wind sensor
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class WindDataMsg : public Message {
public:
	WindDataMsg(NodeID destinationID, NodeID sourceID, uint8_t windDir, uint8_t windSpeed, uint16_t windTemp)
		:Message(MessageType::WindData, sourceID, destinationID),
		m_windDir(windDir), m_windSpeed(windSpeed), m_windTemp(windTemp)
	{ }

	WindDataMsg(uint8_t windDir, uint8_t windSpeed, uint16_t windTemp)
		:Message(MessageType::WindData, NodeID::None, NodeID::None),
		m_windDir(windDir), m_windSpeed(windSpeed), m_windTemp(windTemp)
	{ }

	WindDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readUint16_t(m_windDir) ||
			!deserialiser.readUint8_t(m_windSpeed) ||
			!deserialiser.readUint8_t(m_windTemp))
		{
			m_valid = false;
		}
	}

	virtual ~WindDataMsg() { }

	uint8_t windDirection() { return m_windDir; }
	uint8_t windSpeed() { return m_windSpeed; }
	uint16_t windTemp() { return m_windTemp; }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_windDir);
		serialiser.serialise(m_windSpeed);
		serialiser.serialise(m_windTemp);
	}

private:
	uint16_t 	m_windDir;
	uint8_t 	m_windSpeed;
	uint8_t	  m_windTemp;
};
