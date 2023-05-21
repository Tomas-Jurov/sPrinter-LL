/*
 * ros_serial.h
 *
 *  Created on: May 21, 2023
 *      Author: tomas
 */

#ifndef ROSCONTROL_ROS_SERIAL_H_
#define ROSCONTROL_ROS_SERIAL_H_
#include "../Periph/HardwareSerial.h"
#include "../Periph/SysTickCounter.h"
#include <unistd.h>

namespace ROSControl
{
	class ROSSerial
	{
	public:
		ROSSerial();
		~ROSSerial() = default;
		ssize_t read(uint8_t *buffer, size_t size);
		ssize_t write(const uint8_t *buffer, size_t size);
	private:
		long getElapsedTime(const unsigned long start, const unsigned long end);
	private:
		Periph::SysTickCounter current_time_;
		static constexpr unsigned long timeout_micro_s_ = 50000;
	};
}

#endif /* ROSCONTROL_ROS_SERIAL_H_ */
