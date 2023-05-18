/*
 * Control.h
 *
 *  Created on: 2.9 2018
 *      Author: Zahorack
 */

#ifndef UTIL_CONTROL_H_
#define UTIL_CONTROL_H_

#include "stm32f4xx.h"
#include "Periph/Engine.h"
#include "Periph/Usart.h"
#include "Periph/Servo.h"
#include "Periph/Stepper.h"
#include "Periph/Adc.h"
#include "Util/Packet.h"
#include "Util/Timer.h"
#include "Util/Tool.h"
#include "Device/Suntracker.h"
#include "Periph/Encoder.h"
#include "Periph/EncoderGroup.h"
#include "Util/Pid.h"
#include "Periph/EngineGroup.h"
#include "ROSControl/ros_bridge.h"
#include <cmath>

namespace Util {

class Control {

	Periph::Encoder m_encoders[6] = {
		Periph::Encoder(Periph::EncoderPins::EncoderPin1), Periph::Encoder(Periph::EncoderPins::EncoderPin2),
		Periph::Encoder(Periph::EncoderPins::EncoderPin3), Periph::Encoder(Periph::EncoderPins::EncoderPin4),
		Periph::Encoder(Periph::EncoderPins::EncoderPin5), Periph::Encoder(Periph::EncoderPins::EncoderPin6)
	};

	Periph::Engine m_engines[7] = {
		Periph::Engine(Periph::Engines::M1), Periph::Engine(Periph::Engines::M2),
		Periph::Engine(Periph::Engines::M3), Periph::Engine(Periph::Engines::M4),
		Periph::Engine(Periph::Engines::M5), Periph::Engine(Periph::Engines::M6),
		Periph::Engine(Periph::Engines::M7)
	};

	const Util::PidArgs_t m_pidArgs = {
		.Kp = 0.4,
		.Ki = 1.8,
		.Kd = 0.00005,
		.dt = 0.1,
		.min = 0,
		.max = 99
	};

	Util::Pid m_pids[6] = {
		Util::Pid(&m_pidArgs), Util::Pid(&m_pidArgs), Util::Pid(&m_pidArgs),
		Util::Pid(&m_pidArgs), Util::Pid(&m_pidArgs), Util::Pid(&m_pidArgs)
	};

	Periph::EngineGroup m_rightEngines, m_leftEngines;
	Periph::EncoderGroup m_rightEncoders, m_leftEncoders;
	Periph::Servo 	m_servo1, m_servo2;
	Periph::Stepper m_stepper1, m_stepper2;
	Util::Packet 	m_packet;
	Device::Suntracker m_suntracker;

	Util::Timer	m_timer;
	Util::Timer m_watchdog;
	Util::Tool 	tool;
	ROSControl::ROSBridge m_ros_bridge;
	ROSControl::Returns m_sprinter_state;

	void setRightSideSpeed(uint8_t speed);
	void setLeftSideSpeed(uint8_t speed);
	void setRightSideDirection(Periph::Dirs::Enum dir);
	void setLeftSideDirection(Periph::Dirs::Enum dir);
	void resolveCommands();
	void updateState();

public:
	Control();
	~Control();

	void update();
	void run();
	void stop();
	void start();

	void setWheelsVelocity(int8_t right_vel, int8_t left_vel);
	void updatePrintingData();
	void updateSimulation();
	void taskManager(uint8_t task);

	void updateEncoders();
	void stopEngines();
	void updateEngines();
	void stopServos();
	void startServos();

	void switchMode();
	void test();

};

} /* namespace Util */

#endif /* UTIL_CONTROL_H_ */
