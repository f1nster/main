/* Standard includes. */
#include <string.h>

/* Library includes. */
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

// Подключаем FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// 1-wire
#include "onewire_usart.h"

// dht11
#include "dht11.h"

/* USART port and baud rate used by the echo task. */
#include "STM32_USART.h"
#define mainUSART1				( 0 )
#define mainUSART1_BAUD_RATE	( 115200 )

#define IDS_1WIRE_BUFFER_SIZE	2

//--------------------------------------------------------------


void vTaskLED1(void *pvParameters) {

	for (;;) {
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		vTaskDelay(1000 / portTICK_RATE_MS );
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		vTaskDelay(1000 / portTICK_RATE_MS );
	}
}

void vTaskLED2(void *pvParameters) {

	for (;;) {
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		vTaskDelay(777 / portTICK_RATE_MS );
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		vTaskDelay(777 / portTICK_RATE_MS );
	}
}

static void prvUSARTEchoTask( void *pvParameters ) {

	signed char cChar;

	for( ;; ) {
		/* Block to wait for a character to be received on mainUSART1. */
		xSerialGetChar( mainUSART1, &cChar, portMAX_DELAY );

		/* Write the received character back to mainUSART1. */
		xSerialPutChar( mainUSART1, cChar, 0 );
	}
}

 static void prvUSARTOkTask( void *pvParameters ) {

	 unsigned char i = 0;
	 unsigned char j = 0;

	 for( ;; ) {
		 j++;
		 i = j;
		 xSerialPutChar( mainUSART1, ((i - (i % 100)) / 100 + '0'), 0 );
		 i %= 100;
		 xSerialPutChar( mainUSART1, ((i - (i % 10)) / 10 + '0'), 0 );
		 i %= 10;
		 xSerialPutChar( mainUSART1, (i + '0'), 0 );

		 xSerialPutChar( mainUSART1, ' ', 0 );
		 xSerialPutChar( mainUSART1, 'O', 0 );
		 xSerialPutChar( mainUSART1, 'K', 0 );
		 xSerialPutChar( mainUSART1, '\r', 0 );
		 xSerialPutChar( mainUSART1, '\n', 0 );
		 vTaskDelay(30000 / portTICK_RATE_MS );
	 }
 }

static void prvDS18b20Task( void *pvParameters) {

	uint8_t buf[2];
	uint16_t ulTemp = 0;
	float pfTemp;
	uint8_t temper;
	uint8_t i = 0;

	for(;;) {
		// восстанавливаем функцию передатчика USART
		OW_USART_out_set_as_TX_pin();
		if (OW_USART_Reset() == OW_USART_OK) {
			// Заставляем ВСЕ термометры одновременно провести измерение температуры
			OW_USART_SendOnly(OW_USART_SEND_RESET, (uint8_t *)"\xcc\x44", 2);

			// назначаем функцию двухтактного выхода - подаем "питание" на шину для "паразитного" питания
			OW_USART_out_set_as_Power_pin();

			// выдерживаем время измерения (например 750 мс для 12-битного измерения)
			vTaskDelay(750 / portTICK_RATE_MS );

			// восстанавливаем функцию передатчика USART
			OW_USART_out_set_as_TX_pin();

			// читаем температуру с конкретного датчика
			OW_USART_WriteCmd(OW_MATCH_ROM);
			// адрес датчика с которого читаем
			OW_USART_SendOnly(OW_USART_NO_RESET, (uint8_t *)"\x28\x36\xbb\x5d\x03\x00\x00\xdf", 8);
			OW_USART_Send(OW_USART_NO_RESET, (uint8_t *)"\xbe\xff\xff\xff\xff\xff\xff\xff\xff\xff", 10, buf, 10, 1);

			// читаем температуру с датчика подключенного на линию в одиночестве (предыдущие строки закомментировать)
			// OW_USART_Send(OW_USART_SEND_RESET, (uint8_t *)"\xcc\xbe\xff\xff", 4, buf, 2, 2);

			ulTemp = (u16) buf[0] + (u16) buf[1] * 256;

			if (ulTemp > 2097) {
				ulTemp = 65536 - ulTemp;
				pfTemp = -(((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf) * 0.0625);
			} else {
				pfTemp = ((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf) * 0.0625;
			}

			temper = (u8) pfTemp;
			lSerialPutString( mainUSART1, "\r\n", 2);
			i = ((temper - (temper % 100)) / 100 + '0');
			xSerialPutChar( mainUSART1, i, 0);
			temper %= 100;
			i = ((temper - (temper % 10)) / 10 + '0');
			xSerialPutChar( mainUSART1, i, 0);
			temper %= 10;
			i = (temper + '0');
			xSerialPutChar( mainUSART1, i, 0);

			temper = (u8) (((u16) (pfTemp * 100)) % 100);
			xSerialPutChar( mainUSART1, '.', 0);
			i = ((temper - (temper % 10)) / 10 + '0');
			xSerialPutChar( mainUSART1, i, 0);
			temper %= 10;
			i = (temper + '0');
			xSerialPutChar( mainUSART1, i, 0);
			lSerialPutString( mainUSART1, " - ds18b20\r\n", 12);
		}

		vTaskDelay(5000 / portTICK_RATE_MS );
	}
}

void putc(uint8_t c) {
	if(c < 10)
		xSerialPutChar( mainUSART1, c + '0', 0);
	else
		xSerialPutChar( mainUSART1, c + 'a' - 10, 0);
}

static void prvOneWireScanTask( void *pvParameters ) {

	uint8_t ids_1wire[8*IDS_1WIRE_BUFFER_SIZE];
	uint8_t found = 0;
	uint8_t last_found = 0;
	uint8_t i = 0;
	uint8_t j = 0;

	for( ;; ) {
		// назначаем функцию двухтактного выхода - подаем "питание" на шину для "паразитного" питания
		OW_USART_out_set_as_Power_pin();

		vTaskDelay(100 / portTICK_RATE_MS );

		// восстанавливаем функцию передатчика USART
		OW_USART_out_set_as_TX_pin();

		found = OW_USART_Scan(ids_1wire, IDS_1WIRE_BUFFER_SIZE);
		if ( last_found != found ) {
			lSerialPutString( mainUSART1, "\r\nf=", 4);
			xSerialPutChar( mainUSART1, (found & 0x07) + '0', 0);
			last_found = found;
			for(j=0; j<found; j++) {
				lSerialPutString( mainUSART1, "\r\n+0x", 5);
				for(i = 0; i < 8; i++) {
					putc(ids_1wire[j*8 + i] >> 4);
					putc(ids_1wire[j*8 + i] & 0x0f);
				}
			}
			lSerialPutString( mainUSART1, "\r\n", 2);
		}
	}
}

static void prvDHT11Task( void *pvParameters ) {

	uint8_t humidity;
	uint8_t temperature;

	for( ;; ) {
		vTaskDelay(5000 / portTICK_RATE_MS );

		DHT11_read(&humidity, &temperature);

		lSerialPutString( mainUSART1, "\r\n", 2);
		xSerialPutChar( mainUSART1, ((temperature - (temperature % 100)) / 100 + '0'), 0 );
		temperature %= 100;
		xSerialPutChar( mainUSART1, ((temperature - (temperature % 10)) / 10 + '0'), 0 );
		temperature %= 10;
		xSerialPutChar( mainUSART1, (temperature + '0'), 0 );
		temperature = 0;
		lSerialPutString( mainUSART1, "*C - dht11\r\n", 12);

		xSerialPutChar( mainUSART1, ((humidity - (humidity % 100)) / 100 + '0'), 0 );
		humidity %= 100;
		xSerialPutChar( mainUSART1, ((humidity - (humidity % 10)) / 10 + '0'), 0 );
		humidity %= 10;
		xSerialPutChar( mainUSART1, (humidity + '0'), 0 );
		humidity = 0;
		lSerialPutString( mainUSART1, "%H - dht11\r\n", 12);
	}
}

//--------------------------------------------------------------

int main() {
	// задержка для старта роутера ~30 секунд
	// for(volatile uint32_t i=0;i<(UINT32_MAX/64);i++);

	/* Set the Vector Table base address at 0x08000000. */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	//NVIC_SetPriorityGrouping( 0 );
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
	//!!!!ВАЖНО!!!!
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority -- НЕ ИСПОЛЬЗОВАТЬ -- ВЫЗЫВАЕТ ОШИБКУ ПРИ ПРОВЕРКЕ ЗАДАННЫХ ПРЕРЫВАНИЙ В FreeRTOS

	// leds
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//GPIO_InitTypeDef gpio;
	//gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	//gpio.GPIO_Speed = GPIO_Speed_50MHz;
	//gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_Init(GPIOC, &gpio);
	//GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);

	// Создаем задачи. tskIDLE_PRIORITY - чем больше число - тем выше приоритет
	//xTaskCreate( vTaskLED1, "LED1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	//xTaskCreate( vTaskLED2, "LED2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

	// настройка USART1
	//if (lCOMPortInit( mainUSART1, mainUSART1_BAUD_RATE ) == pdPASS) {
		//xTaskCreate( prvUSARTEchoTask, "Echo", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL );
		//xTaskCreate( prvUSARTOkTask, "OK", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

		//if (OW_USART_Init() == OW_USART_OK) {
			//xTaskCreate( prvOneWireScanTask, "Scan", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
			//xTaskCreate( prvDS18b20Task, "18b20", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 3, NULL);
		//}
		//if (DHT11_Init() == DHT11_OK)
			//xTaskCreate( prvDHT11Task, "dht11", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 1, NULL);
	//}

	// Старт менеджера задач
	vTaskStartScheduler();

	// Сюда попадем в случае переполнения стека
	for (;;) {
	//TODO добавить сюда функционал обработки переполнения стека
		GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);
	}

	return 0;
}

/*-----------------------------------------------------------*/

/*
 * Функции хука Idle ДОЛЖНЫ называться vApplicationIdleHook(), не принимать
 * никаких параметров, и возвращать void.
 * флаг configUSE_IDLE_HOOK
 */
// void vApplicationIdleHook( void ) {
  //TODO добавить сюда функционал подсчета времени простоя
// }

// void vApplicationTickHook(void) {
// }

/*-----------------------------------------------------------*/
