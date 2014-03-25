/*
 * dht11.c
 *
 * Version 1.0.2
 * by mkruglov
 */

#include "dht11.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"

#define DHT11_TIMEOUT_ERR			0xff

#define DHT11_PACK_LEN				40

#define DHT11_PACK_BITS_SEPARATOR	50

#define DHT_START_PULSE_MS			20
#define DHT_PACK_LEN_MS				20

#define DHT_TIM_PERIOD				10000



uint16_t	tickCntLast;
uint16_t	tickCnt;
uint16_t	tickCntDiff;

uint8_t		dht11_buf[DHT11_PACK_LEN];
uint8_t		dht11_buf_pos = 0;

SemaphoreHandle_t	dht11_read_end_semphr;
portBASE_TYPE		dht11_read_end_hptw = pdFALSE;


uint8_t DHT11_Init() {

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(DHT11_PORT_RCC | RCC_APB2Periph_AFIO, ENABLE);

	// Настраиваем EXTI
	GPIO_EXTILineConfig(DHT11_EXTI_PORT_SOURCE, DHT11_EXTI_PIN_SOURCE);
	EXTI_InitStructure.EXTI_Line = DHT11_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Включаем EXTI прерывание
	//!!!! все прерывания объявлять по этому шаблону
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_Init(&NVIC_InitStructure);

	// Настраиваем таймер TIM6
	/* Инициализируем базовый таймер: период 10 мс.
	 * Частота: DHT11_MAX_TICS/сек
	 * Другие параметры структуры TIM_TimeBaseInitTypeDef
	 * не имеют смысла для базовых таймеров.
	 */
	TIM_TimeBaseInitTypeDef base_timer;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructInit(&base_timer);
	/* Делитель учитывается как TIM_Prescaler + 1, поэтому отнимаем 1 */
	// тики таймера каждую μs (микросекунда)
	base_timer.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
	base_timer.TIM_Period = DHT_TIM_PERIOD;
	TIM_TimeBaseInit(TIM6, &base_timer);
	/* Включаем таймер */
	TIM_Cmd(TIM6, ENABLE);

	vSemaphoreCreateBinary( dht11_read_end_semphr );
	if ( dht11_read_end_semphr == NULL)
		return DHT11_FAIL;
	else
		return DHT11_OK;
}

static void DHT11_gpio_input() {
	/* Pin configuration: input floating */
	GPIO_InitTypeDef gpconf = {
		.GPIO_Pin = DHT11_PIN,
		.GPIO_Mode = GPIO_Mode_IN_FLOATING,
		.GPIO_Speed = GPIO_Speed_50MHz,
	};
	GPIO_Init(DHT11_PORT, &gpconf);
}

static void DHT11_gpio_output() {
	/* Pin configuration: output open-drain */
	GPIO_InitTypeDef gpconf = {
		.GPIO_Pin = DHT11_PIN,
		.GPIO_Mode = GPIO_Mode_Out_OD,
		.GPIO_Speed = GPIO_Speed_50MHz,
	};
	GPIO_Init(DHT11_PORT, &gpconf);
}

uint8_t DHT11_read(uint8_t *humidity, uint8_t *temperature) {
	static uint8_t buf[DHT11_PACK_LEN/8];
	uint8_t check_sum;

	// очищаем семафор (берем если есть)
	xSemaphoreTake(dht11_read_end_semphr, 0);

	// Посылаем стартовый сигнал DHT11
	DHT11_gpio_output();
	GPIO_ResetBits(DHT11_PORT, DHT11_PIN);
	vTaskDelay(DHT_START_PULSE_MS / portTICK_RATE_MS );
	GPIO_SetBits(DHT11_PORT, DHT11_PIN);

	// настраиваемся на чтение пакета от датчика
	DHT11_gpio_input();
	// Разрешаем прерывание по внешнему входу
	NVIC_EnableIRQ(EXTI0_IRQn);

	if (xSemaphoreTake(dht11_read_end_semphr, DHT_PACK_LEN_MS / portTICK_RATE_MS) == pdFALSE) {
		dht11_buf_pos = DHT11_TIMEOUT_ERR;
	}

	NVIC_DisableIRQ(EXTI0_IRQn);
	// отпускаем линию
	DHT11_gpio_output();
	GPIO_SetBits( DHT11_PORT, DHT11_PIN );

	if ( dht11_buf_pos == 0 ) {
		// ошибка при чтении пакета
		return DHT11_PACK_ERROR;
	} else if ( dht11_buf_pos == DHT11_TIMEOUT_ERR ) {
		// нет соединения с датчиком
		dht11_buf_pos = 0;
		return DHT11_NO_CONN;
	}

	// конвертируем
	for(dht11_buf_pos = 0; dht11_buf_pos < DHT11_PACK_LEN; dht11_buf_pos++) {
		buf[dht11_buf_pos/8] <<= 1;
		if (dht11_buf[dht11_buf_pos] > DHT11_PACK_BITS_SEPARATOR) {
			buf[dht11_buf_pos/8]++;
		}
	}

	// вычисляем контрольную сумму
	check_sum = 0;
	for(dht11_buf_pos = 0; dht11_buf_pos < 4; dht11_buf_pos++) {
		check_sum += buf[dht11_buf_pos];
	}
	dht11_buf_pos = 0;

	if (buf[DHT11_PACK_LEN/8 -1] != check_sum) return DHT11_CS_ERROR;

	(*humidity) = buf[0];
	(*temperature) = buf[2];
	return DHT11_OK;
}

void DHT11_EXTI_IRQHandler(void)
{
	// Check if DHT11_EXTI_LINE is asserted
	if(EXTI_GetITStatus(DHT11_EXTI_LINE) != RESET)
	{
		tickCntLast = tickCnt;
		tickCnt = TIM_GetCounter(TIM6);

		// определяем начало сообщения
		tickCntDiff = ((tickCnt >= tickCntLast) ? (tickCnt - tickCntLast) : (DHT_TIM_PERIOD - tickCntLast + tickCnt + 1));

		if (dht11_buf_pos != DHT11_TIMEOUT_ERR) {
			if ( ( tickCntDiff > DHT11_PACK_BITS_SEPARATOR * 3 ) && ( tickCntDiff < DHT11_PACK_BITS_SEPARATOR * 4 ) ) {
				// определили начало пакета
				dht11_buf_pos = 1;
			} else if (( (( tickCntDiff > DHT11_PACK_BITS_SEPARATOR ) && ( tickCntDiff < DHT11_PACK_BITS_SEPARATOR * 2 )) ||
				(( tickCntDiff > DHT11_PACK_BITS_SEPARATOR * 2 ) && ( tickCntDiff < DHT11_PACK_BITS_SEPARATOR * 3 )) ) &&
				dht11_buf_pos > 0 ) {
				// считываем биты
				dht11_buf[dht11_buf_pos - 1] = tickCntDiff - DHT11_PACK_BITS_SEPARATOR;
				dht11_buf_pos++;
				if ( dht11_buf_pos > DHT11_PACK_LEN) {
					// пакет прочитан переход к задаче
					dht11_read_end_hptw = pdFALSE;
					xSemaphoreGiveFromISR(dht11_read_end_semphr, &dht11_read_end_hptw);
					portEND_SWITCHING_ISR( dht11_read_end_hptw );
				}
			} else if (dht11_buf_pos > 0) {
				// ошибка при чтении пакета, переход к задаче
				dht11_buf_pos = 0;
				dht11_read_end_hptw = pdFALSE;
				xSemaphoreGiveFromISR(dht11_read_end_semphr, &dht11_read_end_hptw);
				portEND_SWITCHING_ISR( dht11_read_end_hptw );
			}
		}
	}
	// we need to clear line pending bit manually
	EXTI_ClearITPendingBit(DHT11_EXTI_LINE);
}
