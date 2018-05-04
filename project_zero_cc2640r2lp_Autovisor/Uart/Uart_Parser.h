/*
 * Uart_Parser.h
 *
 *  Created on: 29 янв. 2018 г.
 *      Author: CIT_007
 */

#ifndef APPLICATION_UART_PARSER_H_
#define APPLICATION_UART_PARSER_H_

#include <stdint.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>

#define ADR_TX(x)           (x)<<4
#define ADR_REC(x)          (x)
#define ADR_LZO             0x01
#define ADR_PC              0x03

#define GET_STATUS          0x01 //запрос статуса устройства на линии
#define TRANSIT_DATA        0x22 //передача массива данных
#define WRITE_AP            0x23 //запрос ключа шифрования

#define STATUS_OK           0x10 //устройство работает в штатном режиме
#define STATUS_BAD          0x11 //устройство работает некорректно
#define REC_OK              0x12 //подтверждение безошибочного приёма данных
#define REC_ERROR           0x13 //в процессе приёма возникли ошибки
#define NO_COMAND           0x14
#define BAD_CRC             0x15

#define UART_BYTE_DELAY_TIME 5000000
typedef struct {
    uint8_t header;

    uint8_t addr;

    uint8_t data_lenght ;
    uint8_t command;
    uint8_t data[256];
    unsigned short CRC;
} Serial_Data_Packet;

typedef enum {
    stHEADER = 0,
    stAddr,
    stDL,
    stCtrl,
    stDATA,
    stCRC
}State ;


/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned short Crc16(unsigned char*pcBlock, unsigned short len);
 void OnRxByte(unsigned char Chr);
 uint16_t PackProcessing(void);
 void parser_init(void);
// void set_Myaddr(unsigned char addr);
// void set_Masteraddr(unsigned char addr);

#endif /* APPLICATION_UART_PARSER_H_ */
