/*
 * Usart.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: xgallom
 */

#include "../../inc/Periph/HardwareSerial.h"
#include <cmath>
static volatile uint16_t rx_buffer_head_;
static volatile uint16_t rx_buffer_tail_;
static volatile uint16_t tx_buffer_head_;
static volatile uint16_t tx_buffer_tail_;
static volatile bool tx_buffer_is_full_;
static uint8_t rx_buffer_[SERIAL_RX_BUFFER_SIZE];
static uint8_t tx_buffer_[SERIAL_TX_BUFFER_SIZE];


namespace Periph {

void HardwareSerial::initRcc()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

void HardwareSerial::initGpio()
{
	GPIO_InitTypeDef gpioInitStruct = {
			GPIO_Pin: GPIO_Pin_9 | GPIO_Pin_8,
			GPIO_Mode: GPIO_Mode_AF,
			GPIO_Speed: GPIO_Fast_Speed,
			GPIO_OType: GPIO_OType_PP,
			GPIO_PuPd: GPIO_PuPd_UP
	};

	GPIO_Init(GPIOD, &gpioInitStruct);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
}

void HardwareSerial::initUsart(uint32_t baudRate)
{
	USART_InitTypeDef usartInitStruct = {
			USART_BaudRate: baudRate,
			USART_WordLength: USART_WordLength_8b,
			USART_StopBits: USART_StopBits_1,
			USART_Parity: USART_Parity_No,
			USART_Mode: USART_Mode_Rx | USART_Mode_Tx,
			USART_HardwareFlowControl: USART_HardwareFlowControl_None
	};

	USART_Init(USART3, &usartInitStruct);
	USART_Cmd(USART3, ENABLE);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void HardwareSerial::initNvic()
{
	NVIC_InitTypeDef nvicInitStruct = {
			NVIC_IRQChannel: USART3_IRQn,
			NVIC_IRQChannelPreemptionPriority: 0,
			NVIC_IRQChannelSubPriority: 0,
			NVIC_IRQChannelCmd: ENABLE
	};

	NVIC_Init(&nvicInitStruct);
}

HardwareSerial::HardwareSerial(uint8_t port, uint32_t baudRate)
: port_(port)
, current_time_()
{
	initRcc();
	initGpio();
	initUsart(baudRate);
	initNvic();
}

HardwareSerial::~HardwareSerial()
{
	USART_Cmd(USART3, DISABLE);
	NVIC_DisableIRQ(USART3_IRQn);
}

int HardwareSerial::read(void)
{
	if (rx_buffer_head_ == rx_buffer_tail_)
	{
		return -1;
	}
	else
	{
		uint8_t c = rx_buffer_[rx_buffer_tail_];
		rx_buffer_tail_ = (rx_buffer_tail_ + 1) % SERIAL_RX_BUFFER_SIZE;
		return c;
	}
}

size_t HardwareSerial::readBytes(uint8_t *buffer, size_t length)
{
	size_t count = 0;
	while (count < length)
	{
		int c = timedRead();
		if (c < 0) break;
		*buffer++ = (uint8_t)c;
		count++;
	}
	return count;
}

int HardwareSerial::timedRead()
{
	int c;
	unsigned long start = Get_Micros();
	unsigned long end = start;
	do
	{
		c = read();
		end = Get_Micros();
		if (c >= 0) return c;
	} while (std::abs(getElapsedTime(start,end)) > timeout_micro_s_);
	return -1;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
	size_t i;
	for (i = 0; i < size; i++)
	{
		uint16_t tx_buffer_index = (tx_buffer_head_ + 1) % SERIAL_TX_BUFFER_SIZE;

		// Check if the buffer is full
		if (tx_buffer_index == tx_buffer_tail_) {
		  tx_buffer_is_full_ = true;
		  break;
		}

		tx_buffer_[tx_buffer_head_] = buffer[i];
		tx_buffer_head_ = tx_buffer_index;
	}
	// Enable the UART transmit interrupt
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

int HardwareSerial::avaiable(void)
{
	return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + rx_buffer_head_ - rx_buffer_tail_)) % SERIAL_RX_BUFFER_SIZE;
}

inline long HardwareSerial::getElapsedTime(const unsigned long start, const unsigned long end)
{
  return end-start;
}

} /* namespace Periph */

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        // Read the received data
        uint8_t c = USART_ReceiveData(USART3);
        uint16_t rx_buffer_index = (rx_buffer_head_ + 1) % SERIAL_RX_BUFFER_SIZE;

        if (rx_buffer_index != rx_buffer_tail_)
        {
			// Write data to Rx buffer
			rx_buffer_[rx_buffer_head_] = c;
			rx_buffer_head_ = rx_buffer_index;
        }

    }

    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
        // Transmit data from Tx buffer
    	if(!tx_buffer_is_full_ && tx_buffer_head_ != tx_buffer_tail_)
    	{
			USART_SendData(USART3, tx_buffer_[tx_buffer_tail_]);
			tx_buffer_tail_ = (tx_buffer_tail_ + 1) % SERIAL_TX_BUFFER_SIZE;
    	}
    	else
        {
            // Disable the USART transmit interrupt if the buffer is empty
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
