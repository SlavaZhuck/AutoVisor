/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROJECTZERO_H
#define PROJECTZERO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <string.h>
#include <math.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>


/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include <icall.h>

//#include <osal_snv.h>
#include <peripheral.h>
#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "../Uart/Uart_Parser.h"
#include "GeneralDef.h"
// Bluetooth Developer Studio services
#include "vogatt.h"

#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/crypto/CryptoCC26XX.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>


#include <driverlib/vims.h>
#include <driverlib/flash.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>


#include <osal/src/inc/osal_snv.h>
#include "bcomdef.h"

#include "driverlib/aon_batmon.h"
#include "../Uart/Uart_commands.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.

// Struct for messages sent to the application task
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written       */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed    */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value      */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed        */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value    */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing   */
  APP_MSG_SEND_DATA,       /* Request from app to send voice samples to BLE */
  APP_MSG_GET_VOICE_SAMP,
  APP_MSG_Read_name,
  APP_MSG_Write_name,
} app_msg_types_t;

typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;
/*********************************************************************
 * CONSTANTS
 */

#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      0

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   1100
#endif

// Internal Events for RTOS application
#define PRZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PRZ_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define PRZ_STATE_CHANGE_EVT                  Event_Id_00
#define PRZ_CHAR_CHANGE_EVT                   Event_Id_01
#define PRZ_PERIODIC_EVT                      Event_Id_02
#define PRZ_APP_MSG_EVT                       Event_Id_03
#define PRZ_CONN_EVT_END_EVT                  Event_Id_30

#define PRZ_ALL_EVENTS                       (PRZ_ICALL_EVT        | \
                                              PRZ_QUEUE_EVT        | \
                                              PRZ_STATE_CHANGE_EVT | \
                                              PRZ_CHAR_CHANGE_EVT  | \
                                              PRZ_PERIODIC_EVT     | \
                                              PRZ_APP_MSG_EVT      | \
                                              PRZ_CONN_EVT_END_EVT)

/****************************** User's defines **********************************/


#define BUFSIZE                             220

#define MAX_NUM_RX_BYTES    220   // Maximum RX bytes to receive in one go
#define MAX_NUM_TX_BYTES    220   // Maximum TX bytes to send in one go
#define WANTED_RX_BYTES    1   // Maximum TX bytes to send in one go

#define UART_BAUD_RATE 115200

#define NAME_SNV_ID                          BLE_NVID_CUST_START
#define NAME_SNV_SIZE                        BLE_NVID_CUST_START+1

//BLE connection parameters
#define TIMEOUT                             2000
#define MIN_CONNECT_INTERVAL          8
#define MAX_CONNECT_INTERVAL          100

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);


// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void user_Vogatt_ValueChangeHandler(char_data_t *pCharData);
static void user_Vogatt_CfgChangeHandler(char_data_t *pCharData);

// Task handler for sending notifications.
static void user_updateCharVal(char_data_t *pCharData);

// Utility functions
void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType, uint16_t connHandle,
                                       uint16_t serviceUUID, uint8_t paramID,
                                       uint8_t *pValue, uint16_t len );

static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);

/*
 * Task creation function for the Simple BLE Peripheral.
 */
void ProjectZero_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif



void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len);
uint8_t read_BLE_name(uint8_t *name);
uint8_t write_BLE_name(uint8_t *key, uint8_t size);



#endif /* PROJECTZERO_H */
