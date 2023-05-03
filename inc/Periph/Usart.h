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
	void initNvic();

public:
	Usart(Usarts::Enum id, uint32_t baudRate);
	~Usart();

	size_t write(const uint8_t *buffer, uint16_t length);

	size_t read(uint8_t *buffer, uint16_t length);
};

} /* namespace Periph */

extern "C" {
  void USART3_IRQHandler(void);
}

#endif /* PERIPH_USART_H_ */
