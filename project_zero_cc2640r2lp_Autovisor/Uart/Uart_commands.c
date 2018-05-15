/*
 * Uart_commands.c
 *
 *  Created on: 7 февр. 2018 г.
 *      Author: CIT_007
 */
#include <string.h>
#include "Uart_commands.h"
#include "../Application/project_zero_Autovisor.h"
#include "Uart_Parser.h"
#include <ti/drivers/UART.h>
#include "icall_ble_api.h"

//#define KEY_SIZE                            16

 unsigned short calculated_CRC_TX;
extern Serial_Data_Packet Tx_Data;
extern Serial_Data_Packet Rx_Data;
extern UART_Handle uart;
extern uint8_t macAddress[6];
extern uint8_t ble_tx_data[220];
extern unsigned char uart_val;

void calcCRC_andSend(void);
void clear_Tx_packet(void);

void calcCRC_andSend(void){
    calculated_CRC_TX = Crc16((unsigned char*)(&Tx_Data)+1,(unsigned short)(Tx_Data.data_lenght)+3);
    Tx_Data.CRC = calculated_CRC_TX;
    UART_writeCancel(uart);
    UART_write(uart, &Tx_Data, Tx_Data.data_lenght+4);
    UART_write(uart, &Tx_Data.CRC, 2);
}

void send_answer_for_command(uint8_t request){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = request;
    calcCRC_andSend();
}


uint8_t send_data_to_BLE(void){//here we receive data from host
    if(Rx_Data.data_lenght > (220-6))
        Rx_Data.data_lenght = (220-6);

    memcpy(&ble_tx_data, &Rx_Data.data_lenght, sizeof(uint8_t));
    memcpy(&ble_tx_data[1], &Rx_Data.data, Rx_Data.data_lenght);
    //memcpy(&ble_tx_data[2 + Rx_Data.data_lenght], &Rx_Data.CRC, sizeof(unsigned short));
    user_enqueueRawAppMsg(APP_MSG_SEND_DATA, &uart_val, 1);

    //copying data somewhere
    return 1;
}

uint8_t send_data_to_Uart(uint8_t *buf, uint8_t len){

    Tx_Data.header = 0xAA;
    Tx_Data.addr = 0xFF;
    Tx_Data.command = 0x22;
    Tx_Data.data_lenght = len;
    for(uint8_t i = 0 ; i <len; i++)
    {
        Tx_Data.data[i] = buf[i];
    }
    calculated_CRC_TX = Crc16((unsigned char*)(&Tx_Data)+1,(unsigned short)(Tx_Data.data_lenght)+3);
    Tx_Data.CRC = calculated_CRC_TX;
    UART_write(uart, &Tx_Data, Tx_Data.data_lenght+4);
    UART_write(uart, &Tx_Data.CRC, 2);

    return 1;
}

void no_command (void){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = NO_COMAND;
    calcCRC_andSend();
}

void bad_crc(void){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = BAD_CRC;
    calcCRC_andSend();
}

void clear_Tx_packet(void){
    memset(&Tx_Data.data_lenght,0,sizeof(Tx_Data.data_lenght)+sizeof(Tx_Data.command)+sizeof(Tx_Data.data)+sizeof(Tx_Data.CRC));
}
