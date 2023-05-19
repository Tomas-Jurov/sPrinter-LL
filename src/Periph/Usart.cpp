/*
 * Usart.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: xgallom
 */

#include "Periph/Usart.h"
#include <cmath>
namespace Periph {
struct {
	GPIO_TypeDef *gpio;
	uint32_t ahb1Gpio;
	uint32_t rx, tx;
	uint8_t rxSource, txSource, gpioAf;
	USART_TypeDef *usart;
} constexpr config[Usarts::Size] = {
		/* Usart3 */ {
				gpio: GPIOD,
				ahb1Gpio: RCC_AHB1Periph_GPIOD,
				rx: GPIO_Pin_9,
				tx: GPIO_Pin_8,
				rxSource: GPIO_PinSource9,
				txSource: GPIO_PinSource8,
				gpioAf: GPIO_AF_USART3,
				usart: USART3,
		}
};

void Usart::initRcc()
{
	switch(id) {
	case Usarts::Usart3:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		break;

	default: break;
	}

	RCC_AHB1PeriphClockCmd(config[id].ahb1Gpio, ENABLE);
}

void Usart::initGpio()
{
	GPIO_InitTypeDef gpioInitStruct = {
			GPIO_Pin: config[id].rx | config[id].tx,
			GPIO_Mode: GPIO_Mode_AF,
			GPIO_Speed: GPIO_Fast_Speed,
			GPIO_OType: GPIO_OType_PP,
			GPIO_PuPd: GPIO_PuPd_UP
	};

	GPIO_Init(config[id].gpio, &gpioInitStruct);

	GPIO_PinAFConfig(config[id].gpio, config[id].rxSource, config[id].gpioAf);
	GPIO_PinAFConfig(config[id].gpio, config[id].txSource, config[id].gpioAf);
}

void Usart::initUsart(uint32_t baudRate)
{
	// TODO: Make configurable
	USART_InitTypeDef usartInitStruct = {
			USART_BaudRate: baudRate,
			USART_WordLength: USART_WordLength_8b,
			USART_StopBits: USART_StopBits_1,
			USART_Parity: USART_Parity_No,
			USART_Mode: USART_Mode_Rx | USART_Mode_Tx,
			USART_HardwareFlowControl: USART_HardwareFlowControl_None
	};

	USART_Init(config[id].usart, &usartInitStruct);
	USART_Cmd(config[id].usart, ENABLE);
}

Usart::Usart(Usarts::Enum id, uint32_t baudRate) 
: id(id)
, current_time_()
{
	initRcc();
	initGpio();
	initUsart(baudRate);
}

Usart::~Usart()
{
	USART_Cmd(config[id].usart, DISABLE);
}

int Usart::Serial_available()
{
	return USART_GetFlagStatus(config[id].usart, USART_FLAG_RXNE) ? 1 : 0;
}

int Usart::Serial_read()
{
	while (!Serial_available());
	return USART_ReceiveData(config[id].usart);
}


size_t Usart::Serial_readBytes(uint8_t *buffer, size_t length)
{
	ssize_t read_bytes = 0;
	if (!buffer || length <= 0)
	{
		return read_bytes;
	}

	unsigned long start, end;
	start = Get_Micros();
	end = start;

    while (std::abs(getElapsedTime(start,end)) < timeout_micro_s_ && read_bytes < length)
    {
        if (Serial_available() > 0)
        {
        	 buffer[read_bytes++] = USART_ReceiveData(config[id].usart);
        }
        end = Get_Micros();
    }
    return read_bytes;
}

int Usart::Serial_write(const uint8_t *buffer, size_t len)
{
	int i = 0;
    for (; i < len; i++)
    {
        while (!USART_GetFlagStatus(config[id].usart, USART_FLAG_TXE));
        USART_SendData(config[id].usart, buffer[i]);
    }
    return i;
}

void Usart::flushRead()
{
	while (USART_GetFlagStatus(config[id].usart, USART_FLAG_RXNE) == SET)
	{
		USART_ClearFlag(config[id].usart, USART_FLAG_RXNE);
		uint8_t dummy_data = config[id].usart->DR; /* Read data register to clear RXNE flag */
	}
}

ssize_t Usart::read(uint8_t *buffer, size_t length){
	return Serial_readBytes(buffer, (unsigned long)(length));
}

ssize_t Usart::write(const uint8_t *buffer, size_t length)
{
	if (!buffer || length <= 0)
	{
		return 0;
	}
	unsigned long bytesWritten;
	if (!(bytesWritten = Serial_write(buffer,(unsigned long)length)))
	{
		return -1;
	}
	return bytesWritten;
}


inline long Usart::getElapsedTime(const unsigned long start, const unsigned long end)
{
  return end-start;
}

} /* namespace Periph */
