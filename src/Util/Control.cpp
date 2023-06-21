/*
 * Control.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: Zahorack
 */

#include "Util/Control.h"
#include "Application.h"
#include "Util/Timer.h"
#include <string>

namespace Util {

static 	uint8_t s_mode = 0x00;

static constexpr uint8_t KVADRANT_OFFSET = 70;

/*simulacia tlace stvorcovej plochy*/
enum tasks : uint8_t{
  up = 0,
  right,
  down,
  left,

  tasks_size
  };

static uint8_t task = right;

static uint8_t nextTask() {
	const uint8_t rows = 5;

	task++;

	if(task >= tasks_size)
		task = up;

	return task;
}

typedef struct
{
	uint8_t leftEncAngSpeed;
	uint8_t rightEncAngSpeed;
	uint8_t leftEncAngSpeedInScale;
	uint8_t rightEncAngSpeedInScale;
	uint8_t encoderSpeed[6];
	uint8_t leftEngRefSpeed;
	uint8_t rightEngRefSpeed;
	uint8_t leftEngTargSpeed;
	uint8_t rightEngTargSpeed;
	uint8_t rightEngCurrSpeed;
	uint8_t leftEngCurrSpeed;
} monitor_t;

static monitor_t s_data;

void Control::taskManager(uint8_t l_task)
{
	switch(l_task){
		case down:
			m_stepper1.setTargetSteps(1000);
			m_stepper1.setCurrentDirection(Periph::Dirs::Backward);
			break;
		case left:
			m_stepper2.setTargetSteps(1000);
			m_stepper2.setCurrentDirection(Periph::Dirs::Backward);
			break;
		case up:
			m_stepper1.setTargetSteps(1000);
			m_stepper1.setCurrentDirection(Periph::Dirs::Forward);
			break;
		case right:
			m_stepper2.setTargetSteps(1000);
			m_stepper2.setCurrentDirection(Periph::Dirs::Forward);
			break;

	}
}

void Control::updateSimulation()
{
	m_stepper1.start();
	m_stepper2.start();

	if(task == up || task == down){
		if(!m_stepper1.isBusy()){
			taskManager(nextTask());
		}
	}
	else if(task == right || task == left){
		if(!m_stepper2.isBusy())
			taskManager(nextTask());
	}

}

Control::Control():
	m_rightEngines(&m_engines[0], &m_engines[1], &m_engines[2]),
	m_leftEngines(&m_engines[3], &m_engines[4], &m_engines[5]),
	m_rightEncoders(&m_encoders[2], nullptr, nullptr, 1),		// currently, only these encoders are working
	m_leftEncoders(&m_encoders[4], nullptr, nullptr, 1),		// normally, there should be 3 encoders for each side
	m_servo1(Periph::Servos::Servo1),
	m_servo2(Periph::Servos::Servo2),
	m_stepper1(Periph::Steppers::Stepper1),
	m_stepper2(Periph::Steppers::Stepper2),
	m_timer(Util::Time::FromMilliSeconds(25)),
	m_watchdog(Util::Time::FromMilliSeconds(1)),
	m_ros_bridge()
{

	m_watchdog.start();
	m_timer.start();

}


void Control::computeRightSideSpeed()
{

	if (m_rightEngines.getRefSpeed() > 10)
	{
		m_rightEngines.setTargetSpeed(m_pids[0].process(m_rightEngines.getRefSpeed(), m_rightEncoders.getAngularSpeed()));
	} else
	{
		m_rightEngines.setTargetSpeed(0);
	}
}

void Control::computeLeftSideSpeed()
{

	if (m_leftEngines.getRefSpeed() > 10)
	{
		m_leftEngines.setTargetSpeed(m_pids[3].process(m_leftEngines.getRefSpeed(), m_leftEncoders.getAngularSpeed()));
	} else
	{
		m_leftEngines.setTargetSpeed(0);
	}
}

void Control::setRightSideDirection(Periph::Dirs::Enum dir)
{
	m_rightEngines.setTargetDirection(dir);
}

void Control::setLeftSideDirection(Periph::Dirs::Enum dir)
{
	m_leftEngines.setTargetDirection(dir);
}


void Control::updateEngines()
{
	computeRightSideSpeed();
	computeLeftSideSpeed();

	m_engines[0].update();
	m_engines[1].update();
	m_engines[2].update();
	m_engines[3].update();
	m_engines[4].update();
	m_engines[5].update();
	m_engines[6].update();
}

void Control::updateEncoders()
{
	m_encoders[0].update();
	m_encoders[1].update();
	m_encoders[2].update();
	m_encoders[3].update();
	m_encoders[4].update();
	m_encoders[5].update();
}

void Control::updateState()	// doplnit prepocet na jednotky SI podla prevodovych charakteristik
{
	m_leftEngines.getCurrentDirection() == Periph::Dirs::Enum::Forward
		? m_sprinter_state.left_grp_vel = m_leftEncoders.getAngularSpeed()
		: m_sprinter_state.left_grp_vel = -m_leftEncoders.getAngularSpeed();

	m_rightEngines.getCurrentDirection() == Periph::Dirs::Enum::Forward
		? m_sprinter_state.right_grp_vel = m_rightEncoders.getAngularSpeed()
		: m_sprinter_state.right_grp_vel = -m_rightEncoders.getAngularSpeed();

	m_stepper1.getCurrentDirection() == Periph::Dirs::Enum::Forward
		? m_sprinter_state.stepper1_current_steps = m_stepper1.getCurrentSteps()
		: m_sprinter_state.stepper1_current_steps = -m_stepper1.getCurrentSteps();

	m_stepper2.getCurrentDirection() == Periph::Dirs::Enum::Forward
		? m_sprinter_state.stepper2_current_steps = -m_stepper2.getCurrentSteps()
		: m_sprinter_state.stepper2_current_steps = m_stepper2.getCurrentSteps();

	m_sprinter_state.servo1_current_angle = m_servo1.getCurrentAngle();
	m_sprinter_state.servo2_current_angle = m_servo2.getCurrentAngle();

	m_sprinter_state.suntracker_done = true;	// TODO
  m_sprinter_state.tilt_is_on_point = m_tilt_is_on_point;
}

void Control::updateMonitorData()
{
	s_data.leftEncAngSpeed = m_leftEncoders.getAngularSpeed();
	s_data.rightEncAngSpeed = m_rightEncoders.getAngularSpeed();
	s_data.leftEncAngSpeedInScale = m_leftEncoders.getAngularSpeedInScale();
	s_data.rightEncAngSpeedInScale = m_rightEncoders.getAngularSpeedInScale();
	for (int i = 0; i < 6; i++)
	{
		s_data.encoderSpeed[i] = m_encoders[i].getAngularSpeed();
	}
	s_data.leftEngRefSpeed = m_leftEngines.getRefSpeed();
	s_data.rightEngRefSpeed = m_rightEngines.getRefSpeed();
	s_data.leftEngTargSpeed = m_engines[3].getTargetSpeed();
	s_data.rightEngTargSpeed = m_engines[0].getTargetSpeed();
	s_data.leftEngCurrSpeed = m_engines[3].getCurrentSpeed();
	s_data.rightEngCurrSpeed = m_engines[0].getCurrentSpeed();
}

void Control::run()
{
	if(m_timer.run()) {

		update();
	}

	updateEngines();

	updateEncoders();

	updateMonitorData();

	m_stepper1.run();
	m_stepper2.run();

	m_servo1.run();
	m_servo2.run();

	m_suntracker.run();
}

void Control::setWheelsVelocity(int8_t right_vel, int8_t left_vel)
{
  if (right_vel == 0)
  {
	  setRightSideDirection(m_rightEngines.getCurrentDirection());
  }
  else if (right_vel > 0)
  {
	  setRightSideDirection(Periph::Dirs::Forward);
  }
  else
  {
	  setRightSideDirection(Periph::Dirs::Backward);
  }

  if (left_vel == 0)
  {
  	  setLeftSideDirection(m_leftEngines.getCurrentDirection());
  }
  else if (left_vel > 0)
  {
	  setLeftSideDirection(Periph::Dirs::Forward);
  }
  else
  {
	  setLeftSideDirection(Periph::Dirs::Backward);
  }
	m_rightEngines.setRefSpeed(tool.clamp(abs(right_vel), 0, 80));
	m_leftEngines.setRefSpeed(tool.clamp(abs(left_vel), 0, 80));
}

void Control::resolveCommands()
{
	auto result = m_ros_bridge.recieveCommands();

	if (!result)
	{
		auto recieved_data = m_ros_bridge.getRecieved();

    if (recieved_data.command != ROSControl::Command::SET_SPEED_OF_LIN_ACTUATOR)
    {
      m_tilt_is_on_point = false;
    }

		if (recieved_data.command == ROSControl::Command::SET_SPEED_OF_WHEELS)
		{
			setWheelsVelocity(recieved_data.wheels_vel.right, recieved_data.wheels_vel.left);
		}
		else if (recieved_data.command == ROSControl::Command::SET_SPEED_OF_LIN_ACTUATOR)
		{
			recieved_data.tilt_vel > 0
			? m_engines[6].setTargetDirection(Periph::Dirs::Forward)
			: m_engines[6].setTargetDirection(Periph::Dirs::Backward);

      if (recieved_data.tilt_vel == 0)
      {
        m_tilt_is_on_point = true;
      }
      else 
      {
        m_tilt_is_on_point = false;
      }

			m_engines[6].setTargetSpeed(tool.clamp(abs(recieved_data.tilt_vel), 0, 80));
		}
		else if (recieved_data.command == ROSControl::Command::SET_STEPPER1_SPEED)
		{
			m_stepper1.setSpeed(recieved_data.stepper1_speed);
		}
		else if (recieved_data.command == ROSControl::Command::SET_STEPPER2_SPEED)
		{
			m_stepper2.setSpeed(recieved_data.stepper2_speed);
		}
		else if (recieved_data.command == ROSControl::Command::SET_STEPPER1_TARG_STEPS)
		{
			recieved_data.stepper1_target > 0
				? m_stepper1.setCurrentDirection(Periph::Dirs::Forward)
				: m_stepper1.setCurrentDirection(Periph::Dirs::Backward);

			m_stepper1.setTargetSteps(abs(recieved_data.stepper1_target));
			m_stepper1.start();
		}
		else if (recieved_data.command == ROSControl::Command::SET_STEPPER2_TARG_STEPS)
		{
			recieved_data.stepper2_target > 0
				? m_stepper2.setCurrentDirection(Periph::Dirs::Backward)
				: m_stepper2.setCurrentDirection(Periph::Dirs::Forward);
	
			m_stepper2.setTargetSteps(abs(recieved_data.stepper2_target));
			m_stepper2.start();
		}
		else if (recieved_data.command == ROSControl::Command::SET_SERVO1_TARG_ANGLE)
		{
			m_servo1.setTargetAngle(recieved_data.servo1_angle);
		}
		else if (recieved_data.command == ROSControl::Command::SET_SERVO2_TARG_ANGLE)
		{
			m_servo2.setTargetAngle(recieved_data.servo2_angle);
		}
		else if (recieved_data.command == ROSControl::Command::START_SUNTRACKING)
		{
			m_suntracker.enable();
		}
		else if (recieved_data.command == ROSControl::Command::RESET)
		{
			Serial.flushTxBuffer();
		}
	}
}

void Control::update()
{
	resolveCommands();

	updateState();
	m_ros_bridge.setReturns(m_sprinter_state);
	m_ros_bridge.sendReturns();
}


} /* namespace Util */
