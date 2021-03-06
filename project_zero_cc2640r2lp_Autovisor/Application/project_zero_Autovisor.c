/*
 * Filename: project_zero.c
 *
 * Description: This is the simple_central example modified to receive
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 Generated by:
 BDS version: 1.1.3139.0
 Plugin:      Texas Instruments BLE SDK GATT Server plugin 1.0.9
 Time:        Fri Sep 15 2017 01:07:25 GMT+07:00
 *****************************************************************************/

/*
 * INCLUDES
 */
#include <project_zero_Autovisor.h>

/*********************************************************************
 * CONSTANTS
 */


uint8_t ble_tx_data[BUFSIZE];
static uint8_t debug_send = 0;
/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Sync handle globally used to post events to the application thread
static ICall_SyncHandle syncEvent;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Clock structs for periodic notification example
static Clock_Struct v_STREAM_OUTPUT_clock;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)

uint8_t BLE_name[GAP_DEVICE_NAME_LEN] = {'C', 'I', 'T', ' ', 'A', 'U', 'T','O',};
uint8_t BLE_name_size = 9;

//static uint8_t advertData[] =
//{
//  // Flags; this sets the device to use limited discoverable
//  // mode (advertises for 30 seconds at a time) or general
//  // discoverable mode (advertises indefinitely), depending
//  // on the DEFAULT_DISCOVERY_MODE define.
//  0x02,   // length of this data
//  GAP_ADTYPE_FLAGS,
//  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
//
//  // complete name
//  9,
//  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
//  'C', 'I', 'T', ' ', 'A', 'U', 'T','O',
//
//};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CIT AUTO";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;




// Global pin's handle ans status
PIN_Config ledPinTable[] = {
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};


/************************************************ USER'S FUNCTIONS ****************************************/


/*********************************** USER'S VARIABLES ****************************************************/

static int communication_On = 0;

UART_Handle uart;
static UART_Params uartParams;
uint8_t macAddress[6];
static uint32_t wantedRxBytes = WANTED_RX_BYTES;            // Number of bytes received so far
static uint8_t rxBuf[MAX_NUM_RX_BYTES];   // Receive buffer
static uint8_t txBuf[MAX_NUM_RX_BYTES];   // Receive buffer

extern Serial_Data_Packet Tx_Data;
extern Serial_Data_Packet Rx_Data;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// Service callback structure for registering our handlers with the service
// VoGATT callback handler.
// The type VoGATTCBs_t is defined in VoGATT.h
static VogattCBs_t user_VoGATTCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

static void writeCallback(UART_Handle handle_uart, void *txBuf, size_t size)
{
//SPPBLEServer_enqueueUARTMsg(SBC_UART_CHANGE_EVT,rxBuf,size);
}

static void readCallback(UART_Handle handle, void *rxBuf, size_t size)
{
 //   memset(&test_CRC,0,sizeof(test_CRC));

    OnRxByte(((unsigned char*)rxBuf)[0]);
    if(PackProcessing()){

    }

    wantedRxBytes = 1;
    UART_read(handle, rxBuf, wantedRxBytes);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */


static void ProjectZero_init(void)
{



  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &syncEvent);


  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************
 // write_BLE_name(BLE_name, BLE_name_size);
  read_BLE_name(BLE_name);


  uint_least16_t hwiKey = Hwi_disable();

  UART_init();
  parser_init();

// UartLog_init(UART_open(Board_UART0, NULL));
  /* Create a UART with data processing off. */
  UART_Params_init(&uartParams);
  uartParams.writeDataMode    = UART_DATA_BINARY;
  uartParams.readDataMode     = UART_DATA_BINARY;
  uartParams.readMode         = UART_MODE_CALLBACK;
  uartParams.writeMode        = UART_MODE_CALLBACK;
  //uartParams.writeTimeout      = 0; //UART_WAIT_FOREVER
  uartParams.readCallback     = readCallback;
  uartParams.writeCallback    = writeCallback;
  uartParams.readReturnMode   = UART_RETURN_FULL;
  uartParams.readEcho         = UART_ECHO_OFF;
  uartParams.baudRate         = UART_BAUD_RATE;

  uart = UART_open(Board_UART0, &uartParams);
  if (uart == NULL) {
      /* UART_open() failed */
      while (1);
  }

  uint64_t temp = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;
  for(uint8_t i = 0 ; i < 6 ; i++){
      macAddress[i]=*(((uint8_t *)&temp)+i);
  }
  //macAddress = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;

  //UART_write(uart, macAddress, sizeof(macAddress));
  int rxBytes = UART_read(uart, rxBuf, wantedRxBytes);

  Hwi_restore(hwiKey);

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 5000; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  uint8_t advertData[5+GAP_DEVICE_NAME_LEN] = //5+21
  {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) or general
    // discoverable mode (advertises indefinitely), depending
    // on the DEFAULT_DISCOVERY_MODE define.
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // complete name
    9,
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'C', 'I', 'T', ' ', 'A', 'U', 'T','O',
  };

  advertData[3] = BLE_name_size;
  for(uint8_t i = 0; i<BLE_name_size; i++){
      advertData[5+i] = BLE_name[i];
  }

  memset(attDeviceName, 0, sizeof(attDeviceName));
  for(uint8_t i = 0; i<BLE_name_size; i++){
      attDeviceName[i] = BLE_name[i];
  }

  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, 5+BLE_name_size, advertData); //sizeof(5+BLE_name_size-1) //13

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = DEFAULT_PASSCODE;
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

// Add services to GATT server and give ID of this task for Indication acks.
  Vogatt_AddService( selfEntity );

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  Vogatt_RegisterAppCBs( &user_VoGATTCBs );
  // Placeholder variable for characteristic intialization
  uint8_t someVal[20] = {0};

  // Initalization of characteristics in VoGATT that can provide data
  // to a peer device over the air.
  Vogatt_SetParameter(V_STREAM_OUTPUT_ID, V_STREAM_OUTPUT_LEN, &someVal);

  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
  HCI_EXT_SetRxGainCmd(LL_EXT_RX_GAIN_HIGH);
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, PRZ_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for event flags received (event signature 0xffff)
          if (pEvt->signature == 0xffff)
          {
            // Event received when a connection event is completed
            if (pEvt->event_flag & PRZ_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              ProjectZero_sendAttRsp();
            }
          }
          else // It's a message from the stack and not an event.
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }

    }
  }
}

/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */

static void user_processApplicationMessage(app_msg_t *pMsg)
{
    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    switch (pMsg->type)
    {
        case APP_MSG_SERVICE_WRITE: /* Message about received value write */
            /* Call different handler per service */
            switch(pCharData->svcUUID) {
                case VOGATT_SERV_UUID:
                  user_Vogatt_ValueChangeHandler(pCharData);
                break;
            }
        break;

        case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
            /* Call different handler per service */
            switch(pCharData->svcUUID) {
                case VOGATT_SERV_UUID:
                    user_Vogatt_CfgChangeHandler(pCharData);
                break;
            }
        break;

        case APP_MSG_UPDATE_CHARVAL: /* Message to self from to update a value */
            user_updateCharVal(pCharData);
        break;


        case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
            user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
        break;

        case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
        {
            passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
            // Send passcode response.
            GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
        }
        break;


        case APP_MSG_SEND_DATA:
            Vogatt_SetParameter(V_STREAM_OUTPUT_ID, sizeof(ble_tx_data), ble_tx_data);
        break;
        case APP_MSG_Read_name:
            read_BLE_name(BLE_name);
        break;



        case APP_MSG_Write_name:
            if(Rx_Data.data_lenght > GAP_DEVICE_NAME_LEN )
            {
                BLE_name_size = GAP_DEVICE_NAME_LEN;
            }
            else
            {
                BLE_name_size = Rx_Data.data_lenght;
            }

            for(uint8_t i = 0; i < BLE_name_size ; i++){
                BLE_name[i] = Rx_Data.data[i];
            }
            write_BLE_name(BLE_name, BLE_name_size);

        break;
    }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
       }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:

      // Turn off periodic clocks for ind/noti demo
      communication_On = 0;
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:

      // Turn off periodic clocks for ind/noti demo
      communication_On = 0;
      break;

    case GAPROLE_ERROR:
      break;

    default:
      break;
  }
}




/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_Vogatt_ValueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  uint8_t temp_buf[BUFSIZE]={0};
  switch (pCharData->paramID)
  {
    case V_STREAM_START_ID:
      // Do something useful with pCharData->data here

      // -------------------------
      break;

    case V_STREAM_INPUT_ID:  //rx data here!!!
      // Do something useful with pCharData->data here
        memcpy(temp_buf,  pCharData->data, BUFSIZE);
        send_data_to_Uart(&temp_buf[1], temp_buf[0]);
        //memcpy(ble_tx_data, pCharData->data, BUFSIZE);
        //user_enqueueRawAppMsg(APP_MSG_SEND_DATA, &debug_send, 1);
      // -------------------------
      break;

  default:
    return;
  }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_Vogatt_CfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  default:
      break;
  }

  /* Enable notification */
  switch (pCharData->paramID)
  {
    case V_STREAM_OUTPUT_ID:
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ... In the generated example we turn periodic clocks on/off
      if (configValue) // 0x0001 and 0x0002 both indicate turned on.
      {
          if(communication_On != 1){
              GAPRole_SendUpdateParam(MIN_CONNECT_INTERVAL, MAX_CONNECT_INTERVAL, 0, TIMEOUT, GAPROLE_RESEND_PARAM_UPDATE);
              communication_On = 1;
          }
      }
      else
      {
          communication_On = 0;
      }
      break;

  default:
    return;
  }
}


/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   PRZ_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }
  else
  {
    // Got an expected GATT message from a peer.
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}




/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
       // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs,
    .numComparison = numComparison
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
    }
    else
    {
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
    }
  }
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
                          paramID, pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 *  Callbacks from Hwi-context
 *****************************************************************************/


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len )
{
  // Called in Stack's Task context, so can't do processing here.
  // Send message to application message queue about received data.
  uint16_t readLen = len; // How much data was written to the attribute

  // Allocate memory for the message.
  // Note: The pCharData message doesn't have to contain the data itself, as
  //       that's stored in a variable in the service implementation.
  //
  //       However, to prevent data loss if a new value is received before the
  //       service's container is read out via the GetParameter API is called,
  //       we copy the characteristic's data now.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
                                  readLen );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
    pCharData->paramID = paramID;
    // Copy data from service now.
    memcpy(pCharData->data, pValue, readLen);
    // Update pCharData with how much data we received.
    pCharData->dataLen = readLen;
    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
  // Let application know there's a message.
  Event_post(syncEvent, PRZ_APP_MSG_EVT);
  }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.

    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
    Event_post(syncEvent, PRZ_APP_MSG_EVT);
  }
}


/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal(char_data_t *pCharData)
{
  switch(pCharData->svcUUID)
  {
    case VOGATT_SERV_UUID:
      Vogatt_SetParameter( pCharData->paramID,
              pCharData->dataLen, pCharData->data );
    break;

  }
}

/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len)
{
  char        hex[] = "0123456789ABCDEF";
  uint8_t     *pStr = dst;
  uint8_t     avail = dst_len-1;

  memset(dst, 0, avail);

  while (src_len && avail > 3)
  {
    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
    *pStr++ = hex[*src >> 4];
    *pStr++ = hex[*src++ & 0x0F];
    avail -= 2;
    src_len--;
  }

  if (src_len && avail)
    *pStr++ = ':'; // Indicate not all data fit on line.

  return (char *)dst;
}


uint8_t read_BLE_name(uint8_t *name)
{

     uint8_t status = osal_snv_read(NAME_SNV_SIZE, 1, &BLE_name_size);
    status &= osal_snv_read(NAME_SNV_ID, BLE_name_size, name);
    if(status != SUCCESS)
    {
        uint8_t default_BLE_name[] = {'C', 'I', 'T', ' ', 'A', 'U', 'T','O',};
        uint8_t default_BLE_name_size = 9;
        memcpy(name, default_BLE_name, default_BLE_name_size);
    }

    return status;
}

uint8_t write_BLE_name(uint8_t *key, uint8_t size)
{
    uint8_t status = osal_snv_write(NAME_SNV_SIZE, 1, &size);
    status &= osal_snv_write(NAME_SNV_ID, BLE_name_size, key);
    return 1;
}
