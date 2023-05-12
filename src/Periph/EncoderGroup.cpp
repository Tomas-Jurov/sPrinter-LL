/*
 * EncoderGroup.h
 *
 *  Created on: Apr 8, 2023
 *      Author: bugino
 */

#include <Periph/EncoderGroup.h>

namespace Periph {

EncoderGroup::EncoderGroup(Encoder *encoder1, Encoder *encoder2, Encoder *encoder3, uint8_t numOfEncoders)
	: m_encoders{encoder1, encoder2, encoder3},
	  m_numOfEncoders(numOfEncoders)
{}

void EncoderGroup::update()
{
	for(uint8_t n = 0; n < m_numOfEncoders; ++n)
		m_encoders[n]->update();
}

void EncoderGroup::reset()
{
	for(uint8_t n = 0; n < m_numOfEncoders; ++n)
		m_encoders[n]->reset();
}

int32_t EncoderGroup::getDistance()
{
	int32_t distSum = 0;
	for(uint8_t n = 0; n < m_numOfEncoders; ++n)
		distSum += m_encoders[n]->getDistance();
	return distSum / m_numOfEncoders;
}

uint8_t EncoderGroup::getAngularSpeed()
{
	
	uint8_t speedSum = 0;
	for(uint8_t n = 0; n < m_numOfEncoders; ++n)
		speedSum += m_encoders[n]->getAngularSpeed();
	return speedSum / m_numOfEncoders;
}

uint8_t EncoderGroup::getAngularSpeedInScale()
{
	
	uint8_t speedSum = 0;
	for(uint8_t n = 0; n < m_numOfEncoders; ++n)
		speedSum += m_encoders[n]->getAngularSpeedInScale();
	return speedSum / m_numOfEncoders;
}
}
