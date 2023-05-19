/*
 * Usart.h
 *
 *  Created on: Jun 18, 2018
 *      Author: xgallom
 */

#ifndef PERIPH_USART_H_
#define PERIPH_USART_H_

#include "stm32f4xx.h"
#include <cstdio>
#include "Periph/SysTickCounter.h"
namespace Periph {

namespace Usarts {
enum Enum : uint8_t {
	Usart3,
	Size
};
}

class Usart {
	const Usarts::Enum id;

	void initRcc();
	void initGpio();
	void initUsart(uint32_t baudRate);

public:
	Usart(Usarts::Enum id, uint32_t baudRate);
	~Usart();

	int Serial_available();
	int Serial_read();
	int Serial_write(const uint8_t *buffer, size_t len);
	size_t Serial_readBytes(uint8_t *buffer, size_t length);
	void flushRead();

	ssize_t write(const uint8_t *buffer, size_t length);

	ssize_t read(uint8_t *buffer, size_t length);

	long getElapsedTime(const unsigned long start, const unsigned long end);

private:  
	Periph::SysTickCounter current_time_;
	static constexpr unsigned long timeout_micro_s_ = 5000000;
};

} /* namespace Periph */
#endif /* PERIPH_USART_H_ */
