/*
 * ros_serial.cpp
 *
 *  Created on: May 21, 2023
 *      Author: tomas
 */
#include "../../inc/ROSControl/ros_serial.h"
#include <cmath>

namespace ROSControl
{
	ROSSerial::ROSSerial()
	: current_time_()
	{
	}

	ssize_t ROSSerial::read(uint8_t *buffer, size_t size)
	{
		ssize_t read_bytes = 0;
		if (!buffer || size <= 0)
		{
			return read_bytes;
		}

		unsigned long start, end;
		start = Get_Micros();
		end = start;

		unsigned long read_bytes_iteration;

		while (std::abs(getElapsedTime(start, end)) < timeout_micro_s_ && read_bytes < (ssize_t)size)
		{
			if (Serial.avaiable() > 0)
			{
				if (!(read_bytes_iteration = Serial.readBytes(buffer + read_bytes, (size_t)(size - read_bytes))))
				{
					return -1;
				}
			}
			read_bytes += read_bytes_iteration;
			end = Get_Micros();
			read_bytes_iteration = 0;
		}
		return read_bytes;
	}

	ssize_t ROSSerial::write(const uint8_t *buffer, size_t size)
	{
		if (!buffer || size <= 0)
		{
			return 0;
		}

		size_t bytesWritten;
		if (!(bytesWritten = Serial.write(buffer, size)))
		{
			return -1;
		}

		return bytesWritten;
	}

	int ROSSerial::flushRead()
	{
		char dummy;
		while (getChar(&dummy) == 1)
			;
		return 0;
	}

	ssize_t ROSSerial::getChar(char *c)
	{
		if (!c)
		{
			return 0;
		}

		unsigned long bytesRead = 0;
		if(Serial.avaiable()>0){
			if (!(bytesRead = Serial.readBytes((uint8_t *)c, 1)))
			{
				return -1;
			}
		}

		return (ssize_t)bytesRead;
	}

	inline long ROSSerial::getElapsedTime(const unsigned long start, const unsigned long end)
	{
	  return end-start;
	}
}


