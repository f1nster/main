/*
 * dht11.h
 *
 * Работа с датчиком температуры/влажности dht11
 * основано на:
 * https://github.com/tarasii/DTH11.git
 * http://pastebin.com/mY6XXVdR
 *
 * http://www.micro4you.com/files/sensor/DHT11.pdf
 *
 * Использован таймер TIM6 и внешнее прерывание EXTIx
 * Датчик можно посадить на любую ногу микроконтроллера,
 * нужно только настроить прерывание EXTIx на соответствующую ногу.
 *
 * Для использования нужна FreeRTOS
 *
 * Использование:
 *  Перед использованием сделать вызов:
 *   DHT11_Init
 *  Значения с датчика получать функцией:
 *   DHT11_read
 *  Значения температуры (*C) и влажности(%) - целые типа uint8_t (точность температуры 1*C)
 *
 * Version 1.0.2
 * by mkruglov
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DHT11_H
#define DHT11_H

// для использования прерываний и библиотечных типов uint8_t...
#include "stm32f10x.h"

/* Exported constants --------------------------------------------------------*/
#define DHT11_OK			0
#define DHT11_FAIL			1
#define DHT11_NO_CONN		2
#define DHT11_CS_ERROR		3
#define DHT11_PACK_ERROR	4

#define DHT11_PORT GPIOC
#define DHT11_PIN GPIO_Pin_0
#define DHT11_PORT_RCC RCC_APB2Periph_GPIOC
#define DHT11_EXTI_PORT_SOURCE GPIO_PortSourceGPIOC
#define DHT11_EXTI_PIN_SOURCE GPIO_PinSource0 // для GPIO_Pin_0
#define DHT11_EXTI_LINE EXTI_Line0 // для GPIO_Pin_0
#define DHT11_EXTI_IRQHandler EXTI0_IRQHandler


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/*
 * DHT11_Init
 * возвращает DHT11_OK если инициализация удалась инче DHT11_FAIL
 */
uint8_t DHT11_Init();

/*
 * DHT11_read
 * возвращает
 *  DHT11_OK если нет ошибок
 *  DHT11_NO_CONN нет соединения с датчиком
 *  DHT11_CS_ERROR ошибка контрольной суммы
 *  DHT11_PACK_ERROR ошибка длины пакета
 */
uint8_t DHT11_read(uint8_t *humidity, uint8_t *temperature);
void DHT11_EXTI_IRQHandler(void);

#endif /* DHT11_H */
