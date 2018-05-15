/*
 * Uart_commands.h
 *
 *  Created on: 7 ����. 2018 �.
 *      Author: CIT_007
 */

#ifndef APPLICATION_UART_COMMANDS_H_
#define APPLICATION_UART_COMMANDS_H_

#include "Uart_parser.h"

void get_status(void);
uint8_t send_data_to_BLE(void);
uint8_t send_data_to_Uart(uint8_t *buf, uint8_t len);

void no_command(void);
void bad_crc(void);

void send_answer_for_command(uint8_t request);

//void status_ok(void);
//void status_bad(void);
//void rec_ok(void);
//void rec_error(void);
void no_command(void);

#endif /* APPLICATION_UART_COMMANDS_H_ */
