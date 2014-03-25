/*
 * onewire_usart.h
 *
 * Реализация протокола 1-wire с помощью USART по аппноту Maxim
 * http://pdfserv.maximintegrated.com/en/an/AN214.pdf
 *
 * by steel_ne
 * http://we.easyelectronics.ru/STM32/stm32-1-wire-poisk-ustroystv.html
 *
 * Version 1.0.4
 * by mkruglov
 */

#ifndef ONEWIRE_USART_H
#define ONEWIRE_USART_H

// для использования библиотечных типов uint8_t...
#include "stm32f10x.h"

// выбираем, на каком USART находится 1-wire
//#define OW_USART1
#define OW_USART2
//#define OW_USART3
//#define OW_USART4

// если нужно отдавать тики FreeRTOS, то раскомментировать
#define OW_USART_GIVE_TICK_RTOS

// первый параметр функции OW_Send
#define OW_USART_SEND_RESET		1
#define OW_USART_NO_RESET		2

// статус возврата функций
#define OW_USART_OK				1
#define OW_USART_ERROR			2
#define OW_USART_NO_DEVICE		3

#define OW_USART_NO_READ		0xff

#define OW_USART_READ_SLOT		0xff

uint8_t OW_USART_Init();
uint8_t OW_USART_Reset();
uint8_t OW_USART_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_USART_Scan(uint8_t *buf, uint8_t num);
void OW_USART_out_set_as_Power_pin(void);
void OW_USART_out_set_as_TX_pin(void);


// shortcuts for functions
// only send message b wich length is c with RESET flag a
#define OW_USART_SendOnly(a,b,c)  OW_USART_Send(a, b, c, (void*)0, 0, OW_USART_NO_READ)
// send 1 command (with bus reset)
#define OW_USART_WriteCmd(cmd) OW_USART_Send(OW_USART_SEND_RESET, cmd, 1, (void*)0, 0, OW_USART_NO_READ)
// send 1 function (without bus reset)
#define OW_USART_WriteFn(cmd) OW_USART_Send(OW_USART_NO_RESET, cmd, 1, (void*)0, 0, OW_USART_NO_READ)


/*
 * thermometer commands (DS18S20)\
 * send them with bus reset!
 */
// find devices
#define T_SEARCH_ROM		(0xf0)
#define OW_SEARCH_ROM		(uint8_t*)"\xf0"
// read device (when it is alone on the bus)
#define T_READ_ROM			(0x33)
#define OW_READ_ROM			(uint8_t*)"\x33"
// send device ID (after this command - 8 bytes of ID)
#define T_MATCH_ROM			(0x55)
#define OW_MATCH_ROM		(uint8_t*)"\x55"
// broadcast command
#define T_SKIP_ROM			(0xcc)
#define OW_SKIP_ROM			(uint8_t*)"\xcc"
// find devices with critical conditions
#define T_ALARM_SEARCH		(0xec)
#define OW_ALARM_SEARCH		(uint8_t*)"\xec"
/*
 * thermometer functions
 * send them without bus reset!
 */
// start themperature reading
#define T_CONVERT_T			(0x44)
#define OW_CONVERT_T		(uint8_t*)"\x44"
// write critical temperature to device's RAM
#define T_SCRATCHPAD		(0x4e)
#define OW_SCRATCHPAD		(uint8_t*)"\x4e"
// read whole device flash
#define T_READ_SCRATCHPAD	(0xbe)
#define OW_READ_SCRATCHPAD	(uint8_t*)"\xbe"
// copy critical themperature from device's RAM to its EEPROM
#define T_COPY_SCRATCHPAD	(0x48)
#define OW_COPY_SCRATCHPAD	(uint8_t*)"\x48"
// copy critical themperature from EEPROM to RAM (when power on this operation runs automatically)
#define T_RECALL_E2			(0xb8)
#define OW_RECALL_E2		(uint8_t*)"\xb8"
// check whether there is devices wich power up from bus
#define T_READ_POWER_SUPPLY (0xb4)
#define OW_READ_POWER_SUPPLY (uint8_t*)"\xb4"

#endif /* ONEWIRE_USART_H */
