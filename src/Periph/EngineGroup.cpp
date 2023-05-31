/*
 * EngineGroup.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: xgallom
 */

#include <Periph/EngineGroup.h>

namespace Periph {

EngineGroup::EngineGroup(Engine *engine1, Engine *engine2, Engine *engine3)
	: m_engines{engine1, engine2, engine3}
{}

void EngineGroup::start()
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->start();
}

void EngineGroup::stop()
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->stop();
}

void EngineGroup::setTargetSpeed(uint8_t speed)
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->setTargetSpeed(speed);
}

void EngineGroup::setTargetDirection(Periph::Dirs::Enum direction)
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->setTargetDirection(direction);
}

void EngineGroup::setCurrentSpeed(uint8_t speed)
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->setTargetSpeed(speed);
}

void EngineGroup::setRefSpeed(uint8_t speed)
{
	m_refSpeed = speed;
}

uint8_t EngineGroup::getRefSpeed()
{
	return m_refSpeed;
}

void EngineGroup::update()
{
	for(uint8_t n = 0; n < EngineGroup::NumOfEngines; ++n)
		m_engines[n]->update();
}


Periph::Dirs::Enum EngineGroup::getCurrentDirection() const
{
	return m_engines[0]->getCurrentDirection();
}

}
