/*
 * Usart.h
 *
 *  Created on: Jun 18, 2018
 *      Author: xjurov
 */

#ifndef PERIPH_HARDWARESERIAL_H_
#define PERIPH_HARDWARESERIAL_H_
#define SERIAL_RX_BUFFER_SIZE 256
#define SERIAL_TX_BUFFER_SIZE 256

#include "stm32f4xx.h"
#include "SysTickCounter.h"

namespace Periph {

class HardwareSerial {
private:
	void initRcc();
	void initGpio();
	void initUsart(uint32_t baudRate);
	void initNvic();

public:
	HardwareSerial(uint8_t port, uint32_t baudRate);
	~HardwareSerial();

	int avaiable(void);
	int read(void);
	size_t write(uint8_t);
	size_t write(const uint8_t *buffer, size_t size);
	size_t readBytes(uint8_t *buffer, size_t length);
	int timedRead();


	long getElapsedTime(const unsigned long start, const unsigned long end);

private:  
	uint8_t port_;
	Periph::SysTickCounter current_time_;
	static constexpr unsigned long timeout_micro_s_ = 0;
};

} /* namespace Periph */

extern "C" {
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
}

extern Periph::HardwareSerial Serial;

#endif /* PERIPH_HARDWARESERIAL_H_ */
