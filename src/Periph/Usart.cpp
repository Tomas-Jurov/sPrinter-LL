/*
 * Usart.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: xgallom
 */

#include "Periph/Usart.h"

#define RX_BUFFER_SIZE               1024
#define TX_BUFFER_SIZE               1024

namespace Periph {
static volatile char rx_buffer[RX_BUFFER_SIZE];
static volatile char tx_buffer[TX_BUFFER_SIZE];
static volatile int rx_buffer_head = 0;
static volatile int rx_buffer_tail = 0;
static volatile int tx_buffer_head = 0;
static volatile int tx_buffer_tail = 0;

struct {
	GPIO_TypeDef *gpio;
	uint32_t ahb1Gpio;
	uint32_t rx, tx;
	uint8_t rxSource, txSource, gpioAf;
	USART_TypeDef *usart;
	IRQn irqn;
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
				irqn: USART3_IRQn
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
			GPIO_PuPd: GPIO_PuPd_NOPULL
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

	USART_ITConfig(config[id].usart, USART_IT_RXNE, ENABLE);
}

void Usart::initNvic()
{
	NVIC_InitTypeDef nvicInitStruct = {
			NVIC_IRQChannel: static_cast<uint8_t>(config[id].irqn),
			NVIC_IRQChannelPreemptionPriority: 0,
			NVIC_IRQChannelSubPriority: 0,
			NVIC_IRQChannelCmd: ENABLE
	};

	NVIC_Init(&nvicInitStruct);
}

Usart::Usart(Usarts::Enum id, uint32_t baudRate) :
		id(id)
{
	initRcc();
	initGpio();
	initUsart(baudRate);
	initNvic();
}

Usart::~Usart()
{
	USART_Cmd(config[id].usart, DISABLE);
	NVIC_DisableIRQ(config[id].irqn);
}

size_t Usart::write(const uint8_t *buffer, uint16_t length)
{
  int i;

	for (i = 0; i < length; i++) 
  {
    Periph::tx_buffer[Periph::tx_buffer_head] = buffer[i];
    Periph::tx_buffer_head = (Periph::tx_buffer_head + 1) % TX_BUFFER_SIZE; 
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	}

  return i;
}



size_t Usart::read(uint8_t *buffer, uint16_t length)
{
  int i;
	for(i = 0; i < length; i++)
  {
    if (Periph::rx_buffer_head == Periph::rx_buffer_tail)
    {
      break;
    }
    buffer[i] = Periph::rx_buffer[Periph::rx_buffer_tail];
    Periph::rx_buffer_tail = (Periph::rx_buffer_tail + 1) % RX_BUFFER_SIZE; 
	}
	return i;
}


} /* namespace Periph */

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
  {
    Periph::rx_buffer[Periph::rx_buffer_head] = USART_ReceiveData(USART3);
    Periph::rx_buffer_head = (Periph::rx_buffer_head + 1) % RX_BUFFER_SIZE;
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) 
  {
    if (Periph::tx_buffer_head!=Periph::tx_buffer_tail)
    {
      USART_SendData(USART3, Periph::tx_buffer[Periph::tx_buffer_tail]);
      Periph::tx_buffer_tail = (Periph::tx_buffer_tail + 1) % TX_BUFFER_SIZE;
    }
		else 
    {
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
    USART_ClearITPendingBit(USART3, USART_IT_TXE);
	}
}
