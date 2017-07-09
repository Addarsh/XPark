/******************************************************************************

@file  multi_role.c

@brief This file contains the multi_role sample application for use
with the CC2650 Bluetooth Low Energy Protocol Stack.

Group: CMCU, SCS
Target Device: CC2640R2

******************************************************************************

 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************
Release Name: simplelink_cc2640r2_sdk_1_35_00_33
Release Date: 2017-05-02 17:08:44
*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>
#include "hal_trng_wrapper.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Swi.h>
#ifdef USE_CORE_SDK
  #include <ti/display/Display.h>
#else /* !USE_CORE_SDK */
  #include <ti/mw/display/Display.h>
#endif /* USE_CORE_SDK */
#include <ti/mw/lcd/LCDDogm1286.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "multi.h"

#include "board_key.h"
#include "board.h"

#include "two_btn_menu.h"
#include "multi_role_menu.h"
#include "multi_role.h"
#include "aes.h"

/*********************************************************************
* CONSTANTS
*/

//#define DEBUG_TSYNC   //Debug printfs for time syncing

// Maximum number of scan responses
// this can only be set to 15 because that is the maximum
// amount of item actions the menu module supports
#define DEFAULT_MAX_SCAN_RES                  15

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters
#define DEFAULT_CONN_INT                      200
#define DEFAULT_CONN_TIMEOUT                  1000
#define DEFAULT_CONN_LATENCY                  0

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 2000
#define EXTENDED_SCAN_DURATION                2000
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

//Maximum number of scan requests possible
#define DEFAULT_MAX_SCAN_REQ                  10

//Maximum number of Time sync requests possible
#define DEFAULT_MAX_TSYNC_REQ                  10

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

//Manufactuer ID for advertising data
#define MY_MANUFACTURER_ID_1                    0x1633
#define MY_MANUFACTURER_ID_2                    0x1634

//Time sync start message in advert data
#define DEFFAULT_NO_MESSAGE                    0xFF
#define TIME_SYNC_START                        42
#define DIRECT_ADV_CONN                        43
#define ROUTE_DISC                             44

//Index for advert data positions
#define MSG_TYPE_POS                           11
#define ADDR_POS                               16

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #ifdef USE_CORE_SDK
    #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
      #define MR_DISPLAY_TYPE Display_Type_LCD
    #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
      #define MR_DISPLAY_TYPE Display_Type_UART
    #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
      #define MR_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #else // !USE_CORE_SDK
    #if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
      #define MR_DISPLAY_TYPE Display_Type_LCD
    #elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
      #define MR_DISPLAY_TYPE Display_Type_UART
    #else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
      #define MR_DISPLAY_TYPE 0 // Option not supported
    #endif // !BOARD_DISPLAY_EXCLUDE_LCD || !BOARD_DISPLAY_EXCLUDE_UART
  #endif // USE_CORE_SDK
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define MR_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   610
#endif

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define MR_STATE_CHANGE_EVT                  Event_Id_00
#define MR_CHAR_CHANGE_EVT                   Event_Id_01
#define MR_CONN_EVT_END_EVT                  Event_Id_02
#define MR_KEY_CHANGE_EVT                    Event_Id_03
#define MR_PAIRING_STATE_EVT                 Event_Id_04
#define MR_PASSCODE_NEEDED_EVT               Event_Id_05
#define MR_PERIODIC_EVT                      Event_Id_06
#define NOTIFY_ENABLE                        Event_Id_07
#define NOTIFY_CLIENT                        Event_Id_08
#define WRITE_TO_SERVER                      Event_Id_09
#define DELAY_RSP                            Event_Id_10
#define START_SCAN                           Event_Id_11
#define TSYNC_COMPLETE_EVT                   Event_Id_12
#define SCAN_REQ_TIMEOUT                     Event_Id_13
#define DIR_ADV_TIMEOUT                      Event_Id_14
#define START_DIR_ADV                        Event_Id_15
#define START_ROUTE_DISC                     Event_Id_16
#define END_ROUTE_SCAN                       Event_Id_17
#define DISCOVERABLE_NOW                     Event_Id_18
#define NOT_DISCOVERABLE                     Event_Id_19
#define CONNECTION_COMPLETE                  Event_Id_20
#define DISCONNECTED                         Event_Id_21
#define TIMEOUT_WAIT_FOR_CONN                Event_Id_22

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                             MR_QUEUE_EVT            | \
                                             MR_STATE_CHANGE_EVT     | \
                                             MR_CHAR_CHANGE_EVT      | \
                                             MR_CONN_EVT_END_EVT     | \
                                             MR_KEY_CHANGE_EVT       | \
                                             MR_PAIRING_STATE_EVT    | \
                                             MR_PERIODIC_EVT         | \
                                             MR_PASSCODE_NEEDED_EVT  | \
                                             NOTIFY_ENABLE           | \
                                             NOTIFY_CLIENT           | \
                                             WRITE_TO_SERVER         | \
                                             DELAY_RSP               | \
                                             START_SCAN              | \
                                             TSYNC_COMPLETE_EVT      | \
                                             SCAN_REQ_TIMEOUT        | \
                                             DIR_ADV_TIMEOUT         | \
                                             START_DIR_ADV           | \
                                             START_ROUTE_DISC        | \
                                             END_ROUTE_SCAN          | \
                                             DISCOVERABLE_NOW        | \
                                             NOT_DISCOVERABLE        | \
                                             CONNECTION_COMPLETE     | \
                                             DISCONNECTED            | \
                                             TIMEOUT_WAIT_FOR_CONN)

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// Row numbers
#define MR_ROW_DEV_ADDR      (TBM_ROW_APP)
#define MR_ROW_CONN_STATUS   (TBM_ROW_APP + 1)
#define MR_ROW_ADV           (TBM_ROW_APP + 2)
#define MR_ROW_SECURITY      (TBM_ROW_APP + 3)
#define MR_ROW_STATUS1       (TBM_ROW_APP + 4)
#define MR_ROW_STATUS2       (TBM_ROW_APP + 5)

// address string length is an ascii character for each digit +
// an initial 0x + an ending null character
#define B_STR_ADDR_LEN       ((B_ADDR_LEN*2) + 3)

// How often to perform periodic event (in msec)
#define GLOBAL_TIME_CLOCK_PERIOD               1  //1ms clock time
#define SCAN_REQ_DELAY_PERIOD                2000 //1s to wait for scan responses
#define DIR_ADV_PERIOD                       4000 //5s wait time for dir advertising
#define ROUTE_DISC_DELAY                     5000 // 10s wait time for route discovery
#define WAIT_FOR_CONN_DURATION               6000 //6s to wait to connect

//Invalid node info
#define INVALID_NODE_INFO -1
#define VALID_NODE_INFO 16

//Route direc adv max count
#define ROUTE_DIRECT_ADV_MAX_COUNT        1

//Maximum interval for random route discovery
#define ROUTE_DISC_WAIT_MAX_CNT          10
#define ROUTE_DISC_WAIT_MIN_CNT          8
/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData;  // event data pointer
} mrEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; // connection Handle
  uint8_t state;             // state returned from GAPBondMgr
  uint8_t status;            // status of state
} gapPairStateEvent_t;

// discovery information
typedef struct
{
  discState_t discState;   // discovery state
  uint16_t svcStartHdl;    // service start handle
  uint16_t svcEndHdl;      // service end handle
  uint16_t charHdl[5];     //5 handles for 5 characteristics
} discInfo_t;

// device discovery information with room for string address
typedef struct
{
  uint8_t eventType;                // Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
  uint8_t addrType;                 // Address Type: @ref ADDRTYPE_DEFINES
  uint8_t addr[B_ADDR_LEN];         // Device's Address
  uint8_t strAddr[B_STR_ADDR_LEN];  // Device Address as String

  //Additional information for scan requester
  uint8_t bleChan;                  //For scan request information
  int8_t rssi;                      //RSSI value of scan requester
} mrDevRec_t;

//Link for peer node information
typedef struct
{
  int8_t valid;                     //Validity check for the node info
  uint8_t addr[B_ADDR_LEN];         // Device's Address
  uint8_t strAddr[B_STR_ADDR_LEN];  // Device Address as String
  int8_t rssi;                      //RSSI of channel
  //More fields to be added
}mrNode_t;

// entry to map index to connection handle and store address string for menu module
typedef struct
{
  uint16_t connHandle;              // connection handle of an active connection
  uint8_t strAddr[B_STR_ADDR_LEN];  // memory location for menu module to store address string
} connHandleMapEntry_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct passiveAdvertClock;
static Clock_Struct dirAdvClock;
static Clock_Struct routeDiscClock;
static Clock_Struct routeScanClock;
static Clock_Struct waitForConnClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

static uint8_t scanRspData[] =
{
  // complete name
  11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

enum global_state_t{
  TSYNCED_SCANNING,
  TSYNCED_WAIT_FOR_TSYNC_CONN,
  TSYNCED_WAIT_ROUTE_DISC_CLIENT_CONN,
  WAITING_FOR_ROUTE_DISCOVERY,
  TSYNCED_ROUTE_DISCOVERY,
  TSYNCED_ROUTE_CONNECTING,
  WAIITNG_FOR_ROUTE_CONNECT,
  TSYNCED_ROUTE_CONNECTED,
  TSYNCED_ADVERTISING,
  UNSYNCED_SLAVE,
  TSYNCED_MASTER
};

enum unsynced_slave_state_t{
  INIT,
  PASSIVE_ADV,
  DIRECT_ADV,
  CONNECTING,
  CONNECTED
};

static enum global_state_t global_state = UNSYNCED_SLAVE;
//static enum global_state_t global_state = TSYNCED_SCANNING;
static enum unsynced_slave_state_t slave_state = INIT;

//Global clock for time syncronization
static uint64_t my_global_time;

//Route direct adv count
static int route_direct_adv_count = 0;

//Current wait count for route disc
static int route_disc_curr_wait_cnt = 0;

//Time variables used for synchronization of slave
static int64_t T1;
static int64_t T1_prime;
static int64_t T2;
static int64_t T2_prime;

enum tsync_slave_state_t{
  WAIT_FOR_SYNC,
  WAIT_FOR_DELAY_RSP,
  TSYNC_COMPLETE
};

//LED pin states and handles
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

static enum tsync_slave_state_t tsync_slave_state = WAIT_FOR_SYNC;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),

  0x04,   //Type of adv data
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(MY_MANUFACTURER_ID_1),
  HI_UINT16(MY_MANUFACTURER_ID_2),
  DEFFAULT_NO_MESSAGE,

  0x09,     //Direct message addr
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(MY_MANUFACTURER_ID_2),
  HI_UINT16(MY_MANUFACTURER_ID_2),
  DEFFAULT_NO_MESSAGE,
  DEFFAULT_NO_MESSAGE,
  DEFFAULT_NO_MESSAGE,
  DEFFAULT_NO_MESSAGE,
  DEFFAULT_NO_MESSAGE,
  DEFFAULT_NO_MESSAGE
};

// pointer to allocate the connection handle map
static connHandleMapEntry_t *connHandleMap;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Multi Role :)";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pointer to per connection discovery info
discInfo_t *discInfo;

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;

// Scanning started flag
static bool scanningStarted = FALSE;

// Connecting flag
static uint8_t connecting = FALSE;

// Scan result list
static mrDevRec_t devList[DEFAULT_MAX_SCAN_RES];
static int numScanDevs;

//Scan request list (for advertising non-connectable devices)
//List to store the scan request information at one place
static mrDevRec_t scanReqList[DEFAULT_MAX_SCAN_REQ];
static int numScanRequests;

//Time sync request list
//List to store the time sync request information at one place
static mrDevRec_t timeSyncReqList[DEFAULT_MAX_TSYNC_REQ];
static int numTimeSyncRequests;

//Next hop information
static mrNode_t next_hop_node;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = TRUE;

// Dummy parameters to use for connection updates
gapRole_updateConnParams_t updateParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = 80,
  .maxConnInterval = 150,
  .slaveLatency = 0,
  .timeoutMultiplier = 200
};

// Connection index for mapping connection handles
static uint16_t connIndex = INVALID_CONNHANDLE;

// Maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

//Stores my address
static uint8_t myAddr[B_ADDR_LEN];

//Store current peer addr
static uint8_t *curr_peer_addr = NULL;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);
static void multi_role_startDiscovery(uint16_t connHandle);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void multi_role_handleKeys(uint8_t keys);
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);
static void multi_role_keyChangeHandler(uint8_t keysPressed);
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr);
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData);
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void global_time_clockHandler(UArg arg);
static void scanRequest_timeoutHandler(UArg arg);
static void dirAdv_clockHandler(UArg arg);
static void routeDisc_clockHandler(UArg arg);
static void routeScan_clockHandler(UArg arg);
static void  waitForConn_clockHandler(UArg arg);
static bool enable_notifs(uint8_t index);
static uint64_t get_my_global_time(void);
static bool write_to_server(uint8_t index, uint64_t time);
static void perform_time_sync(void);
static void multi_role_processTimeSyncEvts(uint32_t events);
static void addToScanReqReceivedList(hciEvt_BLEScanReqReport_t* scanRequestReport);
static uint8_t *find_target_addr(void);
static bool mr_directConnect(uint8_t addrType, uint8_t* addr);
static bool is_timeSync_req(uint8_t * advData);
static bool is_dirConnAddr_req(uint8_t * advData, int  msg);
static void add_to_timeSync_reqList(gapDeviceInfoEvent_t timeSyncSlave);
static bool saved_time_sync_info(uint8_t *addr);
static void reset_scan_discovery_info(void);

/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,                   // Events to be handled by the app are passed through the GAP Role here
  multi_role_paramUpdateDecisionCB      // Callback for application to decide whether to accept a param update
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  (pfnPasscodeCB_t)multi_role_passcodeCB, // Passcode callback
  multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for multi_role.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, global_time_clockHandler,
                      GLOBAL_TIME_CLOCK_PERIOD, GLOBAL_TIME_CLOCK_PERIOD, false, 0);
  Util_constructClock(&passiveAdvertClock, scanRequest_timeoutHandler,
                             SCAN_REQ_DELAY_PERIOD, 0, false, SCAN_REQ_TIMEOUT);
  Util_constructClock(&dirAdvClock, dirAdv_clockHandler,
                        DIR_ADV_PERIOD, 0, false, DIR_ADV_TIMEOUT);
  Util_constructClock(&routeDiscClock, routeDisc_clockHandler,
                      ROUTE_DISC_DELAY, 0, false, START_ROUTE_DISC);
  Util_constructClock(&routeScanClock, routeScan_clockHandler,
                      DEFAULT_SCAN_DURATION, 0, false, END_ROUTE_SCAN);
  Util_constructClock(&waitForConnClock, waitForConn_clockHandler,
                        WAIT_FOR_CONN_DURATION, 0, false, TIMEOUT_WAIT_FOR_CONN);

  // Init keys and LCD
  Board_initKeys(multi_role_keyChangeHandler);
  // Open Display.
  dispHandle = Display_open(MR_DISPLAY_TYPE, NULL);

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);

  /**************Init Menu*****************************/
  // Disable all menus except mrMenuScan and mrMenuAdvertise
  tbm_setItemStatus(&mrMenuMain, TBM_ITEM_0 | TBM_ITEM_5,
                    TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4 );

  // Disable all items of submenus
  tbm_setItemStatus(&mrMenuConnect, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuGattRw, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuConnUpdate, TBM_ITEM_NONE, TBM_ITEM_ALL);
  tbm_setItemStatus(&mrMenuDisconnect, TBM_ITEM_NONE, TBM_ITEM_ALL);

  // Init two button menu
  tbm_initTwoBtnMenu(dispHandle, &mrMenuMain, 1, NULL);

  // Setup the GAP
  {
    // Set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);

    // Set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

    // Scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);

    // Set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);

    // Register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }

  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = FALSE;

    uint16_t advertOffTime = 0;

    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);

    // Set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData, NULL);

    // Set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

    // set max amount of scan responses
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    // Set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                         &scanRes, NULL);

    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);

    // Allocate memory for index to connection handle map
    if (connHandleMap = ICall_malloc(sizeof(connHandleMapEntry_t) * maxNumBleConns))
    {
      // Init index to connection handle map
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        connHandleMap[i].connHandle = INVALID_CONNHANDLE;
      }
    }

    // Allocate memory for per connection discovery information
    if (discInfo = ICall_malloc(sizeof(discInfo_t) * maxNumBleConns))
    {
      // Init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        discInfo[i].charHdl[SIMPLEPROFILE_CHAR1] = 0;
        discInfo[i].discState = BLE_DISC_STATE_IDLE;
        discInfo[i].svcEndHdl = 0;
        discInfo[i].svcStartHdl = 0;
      }
    }
  }

  // GATT
  {
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    // Initialize GATT Client
    GATT_InitClient();

    // The line masks the RPAO Characteristic in the GAP GATT Server from being
    // detected by remote devices. This value cannot be toggled without power
    // cycling but should remain consistent across power-cycles. Removing this
    // command when Privacy is used will cause this device to be treated in Network
    // Privacy Mode by bonded devices - this means that after disconnecting they
    // will not respond to this device's PDUs which contain its Identity Address.
    GGS_SetParamValue(GGS_DISABLE_RPAO_CHARACTERISTIC);
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

    // Setup Profile Characteristic Values
    {
      uint8_t charValue1 = 1;
      uint8_t charValue2 = 2;
      uint8_t charValue3 = 3;
      uint8_t charValue4 = 4;
      uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                 &charValue1);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                                 &charValue2);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                                 &charValue3);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                 &charValue4);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                 charValue5);
    }

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    // Set pairing mode
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);

    // Set authentication requirements
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);

    // Set I/O capabilities
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);

    // Sst bonding requirements
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

    // Register and start Bond Manager
    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);
  }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & MR_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              multi_role_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      // If RTOS queue is not empty, process app message.
      if (events & MR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            multi_role_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      if(events & START_SCAN){
          Display_print1(dispHandle,12,0,"p: %d", my_global_time);
          if(route_disc_curr_wait_cnt > 0){
            route_disc_curr_wait_cnt--;
            if(route_disc_curr_wait_cnt == 0){
              global_state = TSYNCED_ROUTE_DISCOVERY;
              route_disc_curr_wait_cnt = 0;
            }
          }
      }

       //Manage global application states here
      switch(global_state){
        case TSYNCED_SCANNING:
        case WAITING_FOR_ROUTE_DISCOVERY:
          if(events & START_SCAN){
            //Start scanning
            mr_doScan(0);
            PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
          }else if(events & START_ROUTE_DISC){
            //Starting route discovery
            Display_print0(dispHandle, 15, 0, "Starting route discovery!");

            //Set advertising type to scannable, non-connectable indirect advertising
            uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND;
            GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

            //Set advertising to time-sync
            advertData[MSG_TYPE_POS] = ROUTE_DISC;
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

            global_state = TSYNCED_ROUTE_DISCOVERY;
          }
        break;

        case TSYNCED_WAIT_FOR_TSYNC_CONN:
          if(events & CONNECTION_COMPLETE){
            global_state = TSYNCED_MASTER;
          }
        break;

        case TSYNCED_WAIT_ROUTE_DISC_CLIENT_CONN:
         if(events & CONNECTION_COMPLETE){
           Display_print0(dispHandle, 15,0 ,"I am also route connected!");
         }else if(events & DISCONNECTED){
           Display_print0(dispHandle, 15,0 ,"Going back to previous state!");
           global_state = TSYNCED_SCANNING;
         }
         break;

        case TSYNCED_ROUTE_DISCOVERY:
          if(events & START_SCAN){
            //Turn on advertising
            mr_doAdvertise(0);
          }else if(events & DISCOVERABLE_NOW){
            //Start discovery wait clock
            Util_startClock(&routeScanClock);

            Display_print0(dispHandle, 15,0 ,"Started route discovery non connectable adv!");
          }else if(events & END_ROUTE_SCAN){
            //Turn off advertising
            mr_doAdvertise(0);
          }else if(events & NOT_DISCOVERABLE){

            global_state = TSYNCED_ROUTE_CONNECTING;
          }
        break;

        case TSYNCED_ROUTE_CONNECTING:
          if(events & START_SCAN){
            uint8_t * target_addr = find_target_addr();
            if(target_addr == NULL){

              route_direct_adv_count++;
              Display_print0(dispHandle, 15,0 ,"No target addr found! Go back!");

              if(route_direct_adv_count == ROUTE_DIRECT_ADV_MAX_COUNT){
                //Reset all info
                memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
                numScanRequests = 0;

                global_state = WAITING_FOR_ROUTE_DISCOVERY;
                route_direct_adv_count = 0;
                Display_print0(dispHandle, 15,0 ,"Route busy, try again later!");

                //Set random time to get back to discovery
                uint32_t rand_num = HalTRNG_GetTRNG();
                route_disc_curr_wait_cnt = rand_num % (ROUTE_DISC_WAIT_MAX_CNT + 1 - ROUTE_DISC_WAIT_MIN_CNT)
                                              + ROUTE_DISC_WAIT_MIN_CNT;
                Display_print1(dispHandle, 16, 0, "Random wait count: %d", route_disc_curr_wait_cnt);
              }else{
                global_state = TSYNCED_ROUTE_DISCOVERY;
              }

            }else{
              if(curr_peer_addr == NULL)
                curr_peer_addr = target_addr;

              //Change to connectable indirected advertising
              uint8_t advType = GAP_ADTYPE_ADV_IND;
              GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

              //Change advertising parameters
              advertData[MSG_TYPE_POS] = ROUTE_DISC;
              VOID memcpy(&advertData[ADDR_POS], target_addr, B_ADDR_LEN);
              GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

              mr_doAdvertise(0);

              Display_print1(dispHandle, 15,0 ,"Route direct adv to: %s",
                           (const char*)Util_convertBdAddr2Str(target_addr));
            }
          }
          else if(events & DISCOVERABLE_NOW){
            //Start discovery wait clock
            Util_startClock(&waitForConnClock);

            PIN_setOutputValue(ledPinHandle, Board_RLED, 1);

            global_state = WAIITNG_FOR_ROUTE_CONNECT;
          }
        break;

        case WAIITNG_FOR_ROUTE_CONNECT:
          if(events & TIMEOUT_WAIT_FOR_CONN){
            //stop advertising, try again
            mr_doAdvertise(0);
          }
          else if(events & NOT_DISCOVERABLE){
            PIN_setOutputValue(ledPinHandle, Board_RLED, 0);

            Display_print0(dispHandle, 15,0 ,"Route direct adv timed out!");

            //Reset all info
            memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
            numScanRequests = 0;
            curr_peer_addr = NULL;

            //Reset advertising data
            uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND;
            GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

            //Set advertising to time-sync
            advertData[MSG_TYPE_POS] = DEFFAULT_NO_MESSAGE;
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

            global_state = TSYNCED_ROUTE_DISCOVERY;
          }
          else if(events & CONNECTION_COMPLETE){
            global_state = TSYNCED_ROUTE_CONNECTED;

            Event_post(syncEvent, CONNECTION_COMPLETE);

          }
        break;

        case TSYNCED_ROUTE_CONNECTED:
          if(events & CONNECTION_COMPLETE){
            //Stop advertising
            mr_doAdvertise(0);

            //Save this as next hop address
            next_hop_node.valid = VALID_NODE_INFO;
            VOID memcpy(next_hop_node.addr, curr_peer_addr, B_ADDR_LEN);

            Display_print0(dispHandle, 15,0, "Connected to route!");

            //Green LED to indicate route has been established
            PIN_setOutputValue(ledPinHandle, Board_GLED, 1);

            //Disconnect now
            mr_doDisconnect(0);
          }
          else if(events & DISCONNECTED){
            Display_print1(dispHandle, 15,0, "Route discovery to %s complete!",
                             (const char*)Util_convertBdAddr2Str(next_hop_node.addr));

            //Reset all info
            memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
            numScanRequests = 0;
            curr_peer_addr = NULL;

            //Reset advertising data
            uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND;
            GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

            //Set advertising to time-sync
            advertData[MSG_TYPE_POS] = DEFFAULT_NO_MESSAGE;
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

            global_state = TSYNCED_SCANNING;
          }
        break;

        case TSYNCED_ADVERTISING:

        break;

        case TSYNCED_MASTER:
        case UNSYNCED_SLAVE:
            multi_role_processTimeSyncEvts(events);
        break;
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {
  #if !defined (USE_LL_CONN_PARAM_UPDATE)

              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
  #endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;
          case HCI_LE_EVENT_CODE:
            {
              hciEvt_BLEScanReqReport_t* scanRequestReport = (hciEvt_BLEScanReqReport_t*)pMsg;
              if (scanRequestReport->BLEEventCode == HCI_BLE_SCAN_REQ_REPORT_EVENT)
              {
                //Add device to info list
                addToScanReqReceivedList(scanRequestReport);
              }
            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case GAP_MSG_EVENT:
      multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      break;

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MR_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);

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

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, MR_ROW_STATUS1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, MR_ROW_STATUS1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }
  else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
  {
    //Notification received from the GATT server
    attHandleValueNoti_t *msg_ptr = &(pMsg->msg.handleValueNoti);
    int len = msg_ptr->len;
    memcpy(&T2, msg_ptr->pValue,len);
#ifdef DEBUG_TSYNC
    Display_print1(dispHandle, 12, 0, "T2: %u", T2);
#endif
    Event_post(syncEvent, DELAY_RSP);
  }

  // Messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    // Find index from connection handle
    connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {

      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Write sent: %d", charVal++);
      }
    }
    else if (discInfo[connIndex].discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
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
      multi_role_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp send retry:", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;

  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_KEY_CHANGE_EVT:
    multi_role_handleKeys(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PAIRING_STATE_EVT:
    multi_role_processPairState((gapPairStateEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_PASSCODE_NEEDED_EVT:
    multi_role_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Multi GAPRole event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_paramUpdateDecisionCB
*
* @brief   Callback for application to decide whether or not to accept
*          a parameter update request and, if accepted, what parameters
*          to use
*
* @param   pReq - pointer to param update request
* @param   pRsp - pointer to param update response
*
* @return  none
*/
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp)
{
  // Make some decision based on desired parameters. Here is an example
  // where only parameter update requests with 0 slave latency are accepted
  if (pReq->connLatency == 0)
  {
    // Accept and respond with remote's desired parameters
    pRsp->accepted = TRUE;
    pRsp->connLatency = pReq->connLatency;
    pRsp->connTimeout = pReq->connTimeout;
    pRsp->intervalMax = pReq->intervalMax;
    pRsp->intervalMin = pReq->intervalMin;
  }

  // Don't accept param update requests with slave latency other than 0
  else
  {
    pRsp->accepted = FALSE;
  }
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      // Store max pdu size
      maxPduSize = pEvent->initDone.dataPktLen;

      Display_print0(dispHandle, MR_ROW_DEV_ADDR, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      Display_print0(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to 0");
      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Initialized");

      // Set device info characteristic
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);

      //Configure the Scan Reuest report HCI command
      HCI_EXT_ScanReqRptCmd(HCI_EXT_ENABLE_SCAN_REQUEST_REPORT);

      //Store my address
      GAPRole_GetParameter(GAPROLE_BD_ADDR, &myAddr, NULL);

      //Invalidate next hop node
      next_hop_node.valid = INVALID_NODE_INFO;

      //Start global clock to count time
      Util_startClock(&periodicClock);

    }
    break;

    // Advertising started
    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {
      Display_print0(dispHandle, MR_ROW_ADV, 0, "Advertising");
      Event_post(syncEvent, DISCOVERABLE_NOW);
    }
    break;

    // Advertising ended
    case GAP_END_DISCOVERABLE_DONE_EVENT:
    {
      // Display advertising info depending on whether there are any connections
      if (linkDB_NumActive() < maxNumBleConns)
      {
        Display_print0(dispHandle, MR_ROW_ADV, 0, "Ready to Advertise");
        Event_post(syncEvent, NOT_DISCOVERABLE);
      }
      else
      {
        Display_print0(dispHandle, MR_ROW_ADV, 0, "Can't Adv : Max conns reached");
      }
    }
    break;

    // A discovered device report
    case GAP_DEVICE_INFO_EVENT:
    {
      static int count = 0;
      //Check if directed advertisement, if so, connect immediately
      if((pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND &&
             saved_time_sync_info(pEvent->deviceInfo.addr)
            && is_dirConnAddr_req(pEvent->deviceInfo.pEvtData, DIRECT_ADV_CONN)) ||
              (pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND
                            && is_dirConnAddr_req(pEvent->deviceInfo.pEvtData, ROUTE_DISC))){
#ifdef DEBUG_TSYNC
        Display_print1(dispHandle,18,0,"Target connectable dir adv addr: %s",
                         (const char*)Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
#endif

        //Stop scanning now
        mr_doScan(0);

        //Turn off Red LED
        PIN_setOutputValue(ledPinHandle, Board_RLED, 0);

        if(is_dirConnAddr_req(pEvent->deviceInfo.pEvtData, DIRECT_ADV_CONN)){
          global_state = TSYNCED_WAIT_FOR_TSYNC_CONN;
        }else{
          global_state = TSYNCED_WAIT_ROUTE_DISC_CLIENT_CONN;
        }

        //Reset scan duration
        //GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

        //Connect now to this slave
        mr_directConnect(pEvent->deviceInfo.addrType,pEvent->deviceInfo.addr);

        count++;
        Display_print2(dispHandle,18,0,"Target connectable dir adv addr: %s, count: %d",
                                 (const char*)Util_convertBdAddr2Str(pEvent->deviceInfo.addr),count);
      }else if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_SCAN_IND &&
              is_timeSync_req(pEvent->deviceInfo.pEvtData)){
        add_to_timeSync_reqList(pEvent->deviceInfo);
      }
    }
    break;

    // End of discovery report
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
      uint8_t i;

      // Discovery complete
      scanningStarted = FALSE;

      //Turn off Red LED
      PIN_setOutputValue(ledPinHandle, Board_RLED, 0);

      //Store the number of devices found
      numScanDevs = pEvent->discCmpl.numDevs;

      // If devices were found
      if (pEvent->discCmpl.numDevs > 0)
      {
        // Update menu
        tbm_setItemStatus(&mrMenuMain, TBM_ITEM_1, TBM_ITEM_NONE);
        // Loop through discovered devices to store in static device list
        for (i = 0; i < pEvent->discCmpl.numDevs; i++)
        {
          // Store address type
          devList[i].addrType = pEvent->discCmpl.pDevList[i].addrType;

          // Store event type (adv / scan response)
          devList[i].eventType = pEvent->discCmpl.pDevList[i].eventType;

          // Store address
          memcpy(devList[i].addr, pEvent->discCmpl.pDevList[i].addr, B_ADDR_LEN);

          // Convert address to string
          uint8_t *pAddr = (uint8_t*)Util_convertBdAddr2Str(devList[i].addr);

          // Copy converted string to static device list
          memcpy(devList[i].strAddr, pAddr, B_STR_ADDR_LEN);

          // Set the action description in the connect submenu
          TBM_SET_ACTION_DESC(&mrMenuConnect, i, devList[i].strAddr);
          tbm_setItemStatus(&mrMenuConnect, (1 << i) , TBM_ITEM_NONE);
        }

        // Disable any non-active scan results
        for (; i < (DEFAULT_MAX_SCAN_RES - 1); i++)
        {
          tbm_setItemStatus(&mrMenuConnect, TBM_ITEM_NONE, (1 << i));
        }
      }

      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "Devices Found %d", pEvent->discCmpl.numDevs);
    }
    break;

    // Connection has been established
    case GAP_LINK_ESTABLISHED_EVENT:
    {
      // If succesfully established
      if (pEvent->gap.hdr.status == SUCCESS)
      {

        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connected!");
        Display_print1(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to %d", linkDB_NumActive());

        // Clear connecting flag
        connecting = FALSE;

        // Add index-to-connHandle mapping entry and update menus
        uint8_t index = multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.devAddr);


        //turn off advertising if no available links
        if (linkDB_NumActive() >= maxNumBleConns)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
          Display_print0(dispHandle, MR_ROW_ADV, 0, "Can't adv: no links");
        }

        // Print last connected device
        Display_print0(dispHandle, MR_ROW_STATUS2, 0, (char*)connHandleMap[index].strAddr);

        // Return to main menu
        tbm_goTo(&mrMenuMain);

        // Start service discovery if master
        if(global_state == TSYNCED_WAIT_FOR_TSYNC_CONN){
          multi_role_startDiscovery(pEvent->linkCmpl.connectionHandle);
        }else if(global_state == TSYNCED_WAIT_ROUTE_DISC_CLIENT_CONN){
          multi_role_startDiscovery(pEvent->linkCmpl.connectionHandle);
        }else if(global_state == UNSYNCED_SLAVE){
          slave_state = CONNECTING;
        }
        Event_post(syncEvent, CONNECTION_COMPLETE);
      }
      // If the connection was not successfully established
      else
      {
        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connect Failed");
        Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Reason: %d", pEvent->gap.hdr.status);
      }
    }
    break;

    // Connection has been terminated
    case GAP_LINK_TERMINATED_EVENT:
    {
      //reason for disconnect
      //Display_print1(dispHandle, 13, 0, "Reason for disconnect: %d", pEvent->linkTerminate.reason);
      if(global_state == UNSYNCED_SLAVE && slave_state == DIRECT_ADV){
        //Reset all scan request information
        memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
        slave_state = INIT;
      }

      //Delete scan discovery information as well
      reset_scan_discovery_info();

      // read current num active so that this doesn't change before this event is processed
      uint8_t currentNumActive = linkDB_NumActive();

      // Find index from connection handle
      connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);

      // Check to prevent buffer overrun
      if (connIndex < maxNumBleConns)
      {
        // Clear screen, reset discovery info, and return to main menu
        connHandleMap[connIndex].connHandle = INVALID_CONNHANDLE;

        // Reset discovery info
        discInfo[connIndex].discState = BLE_DISC_STATE_IDLE;
        discInfo[connIndex].charHdl[SIMPLEPROFILE_CHAR1] = 0;

        // Disable connection item from connectable menus
        tbm_setItemStatus(&mrMenuGattRw, TBM_ITEM_NONE, (1 << connIndex));
        tbm_setItemStatus(&mrMenuConnUpdate, TBM_ITEM_NONE, (1 << connIndex));
        tbm_setItemStatus(&mrMenuDisconnect, TBM_ITEM_NONE, (1 << connIndex));

        // If there aren't any active connections
        if (currentNumActive == 0)
        {
          // Disable connectable menus
          tbm_setItemStatus(&mrMenuMain, TBM_ITEM_NONE,
                            TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4);
        }

        // Clear screen
        Display_print1(dispHandle, MR_ROW_CONN_STATUS, 0, "Connected to %d", linkDB_NumActive());
        Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Disconnected!");

        // If it is possible to advertise again
        if (currentNumActive == (maxNumBleConns-1))
        {
          Display_print0(dispHandle, MR_ROW_ADV, 0, "Ready to Advertise");
          Display_print0(dispHandle, MR_ROW_STATUS2, 0, "Ready to Scan");
        }

        if(global_state == TSYNCED_MASTER){
          //Send tsync complete event
          Event_post(syncEvent, TSYNC_COMPLETE_EVT);
        }
        Event_post(syncEvent, DISCONNECTED);
      }
    }
    break;

    // A parameter update has occurred
    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "Param Update %d", pEvent->linkUpdate.status);
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  // Print new value depending on which characteristic was updated
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
    // Get new value
    if(tsync_slave_state == WAIT_FOR_SYNC){
      T1_prime = get_my_global_time();
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &T1);
#ifdef DEBUG_TSYNC
      Display_print2(dispHandle, MR_ROW_STATUS2, 0, "t1: %u, t1_prime: %u", (uint32_t)T1, (uint32_t)T1_prime);
#endif

      //Notify master of T2
      Event_post(syncEvent, NOTIFY_CLIENT);

      tsync_slave_state = WAIT_FOR_DELAY_RSP;
    }else{
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &T2_prime);
#ifdef DEBUG_TSYNC
      Display_print1(dispHandle, MR_ROW_STATUS2+2, 0, "t1 : %u", (uint32_t)T1);
      Display_print1(dispHandle, MR_ROW_STATUS2+3, 0, "t1_prime: %u", (uint32_t)T1_prime);
      Display_print1(dispHandle, MR_ROW_STATUS2+4, 0, "t2: %u", (uint32_t)T2);
      Display_print1(dispHandle, MR_ROW_STATUS2+5, 0, "t2_prime: %u", (uint32_t)T2_prime);
#endif

      perform_time_sync();

      //Disconnect from master
      Event_post(syncEvent, TSYNC_COMPLETE_EVT);
    }
    break;

  case SIMPLEPROFILE_CHAR3:
    // Get new value
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
#ifdef DEBUG_TSYNC
    Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Char 3: %d", (uint16_t)newValue);
#endif
    break;

  case SIMPLEPROFILE_CHAR5:
    //Client config callback situation
#ifdef DEBUG_TSYNC
    Display_print0(dispHandle, 15, 0, "Client has enabled config!");
#endif
    break;

  default:
    // Should not reach here!
    break;
  }
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // Allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // If sucessfully allocated
  if (pMsg)
  {
    // Fill up message
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
void multi_role_keyChangeHandler(uint8_t keys)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    // Store the key data
    *pData = keys;

    // Queue the event.
    multi_role_enqueueMsg(MR_KEY_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void multi_role_handleKeys(uint8_t keys)
{
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(Board_BUTTON0) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(Board_BUTTON1) == 0)
    {
      tbm_buttonRight();
    }
  }
}

/*********************************************************************
* @fn      multi_role_startDiscovery
*
* @brief   Start service discovery.
*
* @param   connHandle - connection handle
*
* @return  none
*/
static void multi_role_startDiscovery(uint16_t connHandle)
{
  // Exchange MTU request
  attExchangeMTUReq_t req;

  // Map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(connHandle);

  // Check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    // Update discovery state of this connection
    discInfo[connIndex].discState= BLE_DISC_STATE_MTU;

    // Initialize cached handles
    discInfo[connIndex].svcStartHdl = discInfo[connIndex].svcEndHdl = 0;
  }

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @param   pMsg - pointer to discovery event stack message
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  // Map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
  // Check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    //MTU update
    if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, MR_ROW_STATUS1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    // If we've updated the MTU size
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_MTU)
    {
      // MTU size response received, discover simple BLE service
      if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
      {
        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
        HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

        // Advance state
        discInfo[connIndex].discState= BLE_DISC_STATE_SVC;

        // Discovery of simple BLE service
        VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid, ATT_BT_UUID_SIZE,
                                           selfEntity);
      }
    }
    // If we're performing service discovery
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_SVC)
    {
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        discInfo[connIndex].svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        discInfo[connIndex].svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }

      // If procedure is complete
      if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
           (pMsg->hdr.status == bleProcedureComplete))  ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        // If we've discovered the service
        if (discInfo[connIndex].svcStartHdl != 0)
        {
          attReadByTypeReq_t req;

               // Discover characteristic
                    discInfo[connIndex].discState = BLE_DISC_STATE_CHAR;
                    req.startHandle = discInfo[connIndex].svcStartHdl;
                    req.endHandle = discInfo[connIndex].svcEndHdl;
                    req.type.len = ATT_BT_UUID_SIZE;
                    req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                    req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

          //VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);

          GATT_DiscAllChars(pMsg->connHandle,discInfo[connIndex].svcStartHdl,
                                discInfo[connIndex].svcEndHdl,selfEntity);
        }
      }
    }
    // If we're discovering characteristics
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_CHAR)
    {
      // Characteristic found
      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
         (pMsg->msg.readByTypeRsp.numPairs > 0))
      {
        int att_len = pMsg->msg.readByTypeRsp.len;
        // Store handle
        for(int i = 0; i < pMsg->msg.readByTypeRsp.numPairs; i++){
          discInfo[connIndex].charHdl[i] = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[att_len*i],
                                                       pMsg->msg.readByTypeRsp.pDataList[att_len*i + 1]);
        }

#ifdef DEBUG_TSYNC
        Display_print0(dispHandle,6,0,"recieved all characteristics!");
#endif
        Event_post(syncEvent, NOTIFY_ENABLE);
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_mapConnHandleToIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
* @return  index or INVALID_CONNHANDLE if connHandle isn't found
*/
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  // Loop through connection
  for (index = 0; index < maxNumBleConns; index ++)
  {
    // If matching connection handle found
    if (connHandleMap[index].connHandle == connHandle)
    {
      return index;
    }
  }
  // Not found if we got here
  return INVALID_CONNHANDLE;
}

/************************************************************************
* @fn      multi_role_pairStateCB
*
* @param   connHandle - the connection handle
*
* @param   state - pairing state
*
* @param   status - status of pairing state
*
* @return  none
*/
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  gapPairStateEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
  {
    pData->connectionHandle = connHandle;
    pData->state = state;
    pData->status = status;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PAIRING_STATE_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_processPairState
*
* @brief   Process the new paring state.
*
* @param   pairingEvent - pairing event received from the stack
*
* @return  none
*/
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent)
{
  // If we've started pairing
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print1(dispHandle, MR_ROW_SECURITY, 0,"connHandle %d pairing", pairingEvent->connectionHandle);
  }
  // If pairing is finished
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0,"connHandle %d paired", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, MR_ROW_SECURITY, 0, "pairing failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  // If a bond has happened
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  // If a bond has been saved
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      Display_print2(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  // Use static passcode
  uint32_t passcode = 123456;
  Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Passcode: %d", passcode);
  // Send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      global_time_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 */
static void global_time_clockHandler(UArg arg)
{
  //Increment global time
  my_global_time++;
  if(my_global_time % 4000 == 0){
    Event_post(syncEvent, START_SCAN);
  }
}

/*********************************************************************
 * @fn      dirAdv_clockHandler
 *
 * @brief   Handler function for direct advertising timeouts
 *
 * @param   arg - event type
 */
static void dirAdv_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      routeDisc_clockHandler
 *
 * @brief   Handler function for route discovery
 *
 * @param   arg - event type
 */
static void routeDisc_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      routeScan_clockHandler
 *
 * @brief   Handler function for route Scanning
 *
 * @param   arg - event type
 */
static void routeScan_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      waitForConn_clockHandler
 *
 * @brief   Handler function to wait for connection
 *
 * @param   arg - event type
 */
static void  waitForConn_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      scanRequest_timeoutHandler
 *
 * @brief   Handler function for Scan request timeouts
 *
 * @param   arg - event type
 */
static void scanRequest_timeoutHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
* @fn      multi_role_addMappingEntry
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr)
{
  uint16_t index;
  // Loop though connections
  for (index = 0; index < maxNumBleConns; index++)
  {
    // If there is an open connection
    if (connHandleMap[index].connHandle == INVALID_CONNHANDLE)
    {
      // Store mapping
      connHandleMap[index].connHandle = connHandle;

      // Convert address to string
      uint8_t *pAddr = (uint8_t *) Util_convertBdAddr2Str(addr);

      // Copy converted string to persistent connection handle list
      memcpy(connHandleMap[index].strAddr, pAddr, B_STR_ADDR_LEN);

      // Enable items in submenus
      tbm_setItemStatus(&mrMenuGattRw, (1 << index), TBM_ITEM_NONE);
      tbm_setItemStatus(&mrMenuConnUpdate, (1 << index), TBM_ITEM_NONE);
      tbm_setItemStatus(&mrMenuDisconnect, (1 << index), TBM_ITEM_NONE);

      // Add device address as a string to action description of connectable menus
      TBM_SET_ACTION_DESC(&mrMenuGattRw, index, connHandleMap[index].strAddr);
      TBM_SET_ACTION_DESC(&mrMenuConnUpdate, index, connHandleMap[index].strAddr);
      TBM_SET_ACTION_DESC(&mrMenuDisconnect, index, connHandleMap[index].strAddr);

      // Enable connectable menus if they are disabled
      if (!(TBM_IS_ITEM_ACTIVE(&mrMenuMain, TBM_ITEM_2)))
      {
        tbm_setItemStatus(&mrMenuMain, TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4,
                          TBM_ITEM_NONE);
      }

      return index;
    }
  }
  // No room if we get here
  return bleNoResources;
}

/*********************************************************************
* @fn      mr_doScan
*
* @brief   Respond to user input to start scanning
*
* @param   index - not used
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doScan(uint8_t index)
{
  (void) index;

  // If we can connect to another device
  if (linkDB_NumActive() < maxNumBleConns)
  {
    // If we're not already scanning
    if (!scanningStarted)
    {
      // Set scannin started flag
      scanningStarted = TRUE;

      // Start scanning
      GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                             DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);

      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Discovering...");
    }
    // We're already scanning...so cancel
    else
    {
      // Cancel scanning
      GAPRole_CancelDiscovery();
      Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Discovery Cancelled");

      // Clear scanning started flag
      scanningStarted = FALSE;
    }
  }
  return TRUE;
}

/*********************************************************************
* @fn      mr_doConnect
*
* @brief   Respond to user input to form a connection
*
* @param   index - index as selected from the mrMenuConnect
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doConnect(uint8_t index)
{
  // If already connecting...cancel
  if (connecting == TRUE)
  {
    // Cancel connection request
    GAPRole_TerminateConnection(GAP_CONNHANDLE_INIT);
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting Cancelled");

    // Clear connecting flag
    connecting = FALSE;
  }
  // If attempting to connect
  else
  {
    // Connect to current device in scan result
    GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                          DEFAULT_LINK_WHITE_LIST,
                          devList[index].addrType, devList[index].addr);

    // Set connecting state flag
    connecting = TRUE;
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting to:");
    Display_print0(dispHandle, MR_ROW_STATUS2, 0, (char*)devList[index].strAddr);
  }

  return TRUE;
}

//Function to respond to a direct connection
static bool mr_directConnect(uint8_t addrType, uint8_t* addr)
{
  // Connect to current device in scan result
  GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                          DEFAULT_LINK_WHITE_LIST,
                          addrType, addr);

  // Set connecting state flag
  Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting to:");
  Display_print0(dispHandle, MR_ROW_STATUS2, 0, (char*)Util_convertBdAddr2Str(addr));
  connecting =  TRUE;

  return TRUE;
}

/*********************************************************************
* @fn      mr_doGattRw
*
* @brief   Respond to user input to do a GATT read or write
*
* @param   index - index as selected from the mrMenuGattRw
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doGattRw(uint8_t index)
{
  bStatus_t status = FAILURE;
  // If characteristic has been discovered
  Display_print1(dispHandle,20,0,"conn map index: %d", index);
  if (discInfo[index].charHdl[SIMPLEPROFILE_CHAR1] != 0)
  {
    // Do a read / write as long as no other read or write is in progress
    if (doWrite)
    {
      // Do a write
      attWriteReq_t req;

      // Allocate GATT write request
      req.pValue = GATT_bm_alloc(connHandleMap[index].connHandle, ATT_WRITE_REQ, 1, NULL);
      // If successfully allocated
      if (req.pValue != NULL)
      {
        // Fill up request
        req.handle = discInfo[index].charHdl[SIMPLEPROFILE_CHAR1];
        req.len = 1;
        req.pValue[0] = charVal;
        req.sig = 0;
        req.cmd = 0;

        // Send GATT write to controller
        status = GATT_WriteCharValue(connHandleMap[index].connHandle, &req, selfEntity);

        // If not sucessfully sent
        if ( status != SUCCESS )
        {
          // Free write request as the controller will not
          GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        }
      }
    }
    // Do a read
    else
    {
      // Create read request...place in CSTACK
      attReadReq_t req;

      // Fill up read request
      req.handle = discInfo[index].charHdl[SIMPLEPROFILE_CHAR1];

      // Send read request. no need to free if unsuccessful since the request
      // is only placed in CSTACK; not allocated
      status = GATT_ReadCharValue(connHandleMap[index].connHandle, &req, selfEntity);
    }

    // If succesfully queued in controller
    if (status == SUCCESS)
    {
      // Toggle read / write
      //doWrite = !doWrite;
    }
  }

  return TRUE;
}

/*********************************************************************
* @fn      mr_doConnUpdate
*
* @brief   Respond to user input to do a connection update
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doConnUpdate(uint8_t index)
{
  bStatus_t status = FAILURE;
  // Fill in connection handle in dummy params
  updateParams.connHandle = connHandleMap[index].connHandle;

  // Send connection parameter update
  status = gapRole_connUpdate( GAPROLE_NO_ACTION, &updateParams);

  // If successfully sent to controller
  if (status == SUCCESS)
  {
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Updating");
  }
  // If there is already an ongoing update
  else if (status == blePending)
  {
    Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Already Updating");
  }

  return TRUE;
}

/*********************************************************************
* @fn      mr_doDisconnect
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool mr_doDisconnect(uint8_t index)
{
  // Disconnect
  GAPRole_TerminateConnection(connHandleMap[index].connHandle);
  Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Disconnecting");

  return TRUE;
}

/* Actions for Menu: Init - Advertise */
bool mr_doAdvertise(uint8_t index)
{
  (void) index;
  uint8_t adv;
  uint8_t adv_status;

  // Get current advertising status
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);

  // If we're currently advertising
  if (adv_status)
  {
    // Turn off advertising
    adv = FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
  }
  // If we're not currently advertising
  else
  {
    // Turn on advertising
    adv = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
  }

  return TRUE;
}

//Enable notifications for server
static bool enable_notifs(uint8_t index)
{
  bStatus_t status = FAILURE;
  // If characteristic has been discovered
  if (discInfo[index].charHdl[SIMPLEPROFILE_CHAR1] != 0)
  {
      // Do a write
      attWriteReq_t req;

      // Allocate GATT write request
      req.pValue = GATT_bm_alloc(connHandleMap[index].connHandle, ATT_WRITE_REQ, 2, NULL);
      // If successfully allocated
      if (req.pValue != NULL)
      {
        // Fill up request
        req.handle = discInfo[index].charHdl[SIMPLEPROFILE_CHAR4] + 2;  //Notif handle is 1 ahead of char val handle
        req.len = 2;
        req.pValue[0] = 0x01;
        req.pValue[1] = 0x00;
        req.sig = 0;
        req.cmd = 0;

        // Send GATT write to controller
        status = GATT_WriteCharValue(connHandleMap[index].connHandle, &req, selfEntity);

        // If not sucessfully sent
        if ( status != SUCCESS )
        {
          // Free write request as the controller will not
          GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);

        }
      }
  }
  return status;
}

/* Returns the current global time in milliseconds */
static uint64_t get_my_global_time(void)
{
  uint64_t curr_time;

  uint_t key = Swi_disable();
  curr_time = my_global_time;
  Swi_restore(key);

  return curr_time;
}

static bool write_to_server(uint8_t index, uint64_t time)
{
  bStatus_t status = FAILURE;

  if (discInfo[index].charHdl[SIMPLEPROFILE_CHAR1] != 0)
  {
    // Do a write
    attWriteReq_t req;

    // Allocate GATT write request
    req.pValue = GATT_bm_alloc(connHandleMap[index].connHandle, ATT_WRITE_REQ, SIMPLEPROFILE_CHAR1_LEN, NULL);
    // If successfully allocated
    if (req.pValue != NULL)
    {
      // Fill up request
      req.handle = discInfo[index].charHdl[SIMPLEPROFILE_CHAR1] + 1;
      req.len = SIMPLEPROFILE_CHAR1_LEN;
      VOID memcpy(req.pValue, &time, SIMPLEPROFILE_CHAR1_LEN);
      req.sig = 0;
      req.cmd = 0;

      // Send GATT write to controller
      status = GATT_WriteCharValue(connHandleMap[index].connHandle, &req, selfEntity);

      // If not sucessfully sent
      if ( status != SUCCESS )
      {
        // Free write request as the controller will not
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
      }
    }
  }

  return status;
}

//This function will synchroize the server
//global time with the client global time
//using the precision time procotol algorithm
static void perform_time_sync(void)
{
  int64_t offset = -(T1_prime - T1 -T2_prime + T2)/2;

  uint_t key = Swi_disable();
  my_global_time += offset;
  Swi_restore(key);
}

//Handle all time synchronization events here
static void multi_role_processTimeSyncEvts(uint32_t events)
{
  if(global_state == TSYNCED_MASTER){

    if (events & NOTIFY_ENABLE){
      //Write to enable configuration callback
      if(enable_notifs(0) == FAILURE){
        Event_post(syncEvent, NOTIFY_ENABLE);
      }else{
        //Send time to T1
        Event_post(syncEvent, WRITE_TO_SERVER);
       }
     }
    else if(events & WRITE_TO_SERVER){
       //Notify the client, call SetParameter
       T1 = get_my_global_time();
       if(write_to_server(0,T1) == FAILURE){
#ifdef DEBUG_TSYNC
         Display_print0(dispHandle,15,0,"Error! Write to server failed!");
#endif
         Event_post(syncEvent, WRITE_TO_SERVER);
       }else{
#ifdef DEBUG_TSYNC
         Display_print0(dispHandle,15,0,"Success! Wrote to server!");
#endif
       }
     }
    else if(events & DELAY_RSP){
      //This is the delay response message
      T2_prime = get_my_global_time();
      if(write_to_server(0,T2_prime) == FAILURE){
        Display_print0(dispHandle,15,0,"Error! Write to server failed!");
        Event_post(syncEvent, WRITE_TO_SERVER);
      }else{
#ifdef DEBUG_TSYNC
        Display_print0(dispHandle,15,0,"Success! Wrote to server!");
#endif
      }
    }
    else if(events & TSYNC_COMPLETE_EVT){
      //Reset time sync req list
      memset(timeSyncReqList, 0, numTimeSyncRequests*sizeof(mrDevRec_t));
      numTimeSyncRequests = 0;

      //If the no links exist, initiate route discovery
      if(next_hop_node.valid == INVALID_NODE_INFO)
        Util_startClock(&routeDiscClock);

      //Tsync is complete and host has been disconnected
      global_state = TSYNCED_SCANNING;
    }
  }
  else if(global_state == UNSYNCED_SLAVE){
    uint8_t advType;
    switch(slave_state){
      case INIT:
        //Reset scanReqList
        memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
        numScanRequests = 0;

        //Set advertising type to scannable, non-connectable indirect advertising
        advType = GAP_ADTYPE_ADV_SCAN_IND;
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

        //Set advertising to time-sync
        advertData[MSG_TYPE_POS] = TIME_SYNC_START;
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

        //Turn on advertising
        mr_doAdvertise(0);

        //Turn on Red LED
        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);

        slave_state = PASSIVE_ADV;
        break;

      case PASSIVE_ADV:
        if(events & DISCOVERABLE_NOW){
          //Start clock for passive advertising
          Util_startClock(&passiveAdvertClock);
        }else if(events & SCAN_REQ_TIMEOUT){
          //Turn off advertising
          mr_doAdvertise(0);
        }else if(events & NOT_DISCOVERABLE){
          //End of being discoverable, start direct advertising
          slave_state = DIRECT_ADV;
          Event_post(syncEvent, START_DIR_ADV);
        }
        break;

      case DIRECT_ADV:
        if(events & START_DIR_ADV){

          uint8_t *target_addr = find_target_addr();
          if(target_addr == NULL){
#ifdef DEBUG_TSYNC
            Display_print0(dispHandle,20,0,"No target address found!");
#endif
            slave_state = INIT;
            return;
          }

          //Change to connectable indirected advertising
          uint8_t advType = GAP_ADTYPE_ADV_IND;
          GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

          //Change advertising parameters
          advertData[MSG_TYPE_POS] = DIRECT_ADV_CONN;
          VOID memcpy(&advertData[ADDR_POS], target_addr, B_ADDR_LEN);
          GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

          //Start advertising again
          mr_doAdvertise(0);
#ifdef DEBUG_TSYNC
          Display_print1(dispHandle,20,0,"direct advertising has started to: %s!",
                         (const char*)Util_convertBdAddr2Str(target_addr));
#endif
        }
        else if(events & DISCOVERABLE_NOW){
          //Start direct advertisement clock for dir advertising
          Util_startClock(&dirAdvClock);
        }
        else if(events & DIR_ADV_TIMEOUT){
          //Return back to INIT
#ifdef DEBUG_TSYNC
          Display_print0(dispHandle,20,0,"direct advertising has timed out!");
#endif

          //Turn off advertising
          mr_doAdvertise(0);
        }else if(events & NOT_DISCOVERABLE){
          //Go back to initial state
          slave_state = INIT;
        }
        break;

      case CONNECTING:
        if(events & CONNECTION_COMPLETE){
            //Stop advertising
            mr_doAdvertise(0);
        }
        else if(events & NOT_DISCOVERABLE){
          tsync_slave_state = WAIT_FOR_SYNC;
          slave_state = CONNECTED;
        }
      break;

      case CONNECTED:
        if (events & NOT_DISCOVERABLE){
          //Turn off Red LED
          PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        }
        else if(events & NOTIFY_CLIENT){
          //Notify the client, call SetParameter
          T2 = get_my_global_time();
          if(SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,SIMPLEPROFILE_CHAR4_LEN,&T2) == FAILURE){
            Display_print0(dispHandle, 15, 0, "Error! Notification send failed!");
          }
        }
        else if (events & TSYNC_COMPLETE_EVT){
          //Disconnect and change state
          mr_doDisconnect(0);

          //If the no links exist, initiate route discovery
          if(next_hop_node.valid == INVALID_NODE_INFO)
            Util_startClock(&routeDiscClock);

          //Reset scanReqList
          memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
          numScanRequests = 0;

          slave_state = INIT;
          tsync_slave_state = WAIT_FOR_SYNC;
          global_state = TSYNCED_SCANNING;
        }
        break;
    }
  }
}

//Add scan requester information to list
static void addToScanReqReceivedList(hciEvt_BLEScanReqReport_t* scanRequestReport)
{
  if(numScanRequests == DEFAULT_MAX_SCAN_REQ)
    return;
  uint8_t *peerAddr = scanRequestReport->peerAddr;
  for(int i = 0; i < numScanRequests; i++){
    if(memcmp(peerAddr, &(scanReqList[i].addr), B_ADDR_LEN)== 0){
      //Device already exists in the list
      return;
    }
  }

  //Store Scan Requester information
  memcpy(scanReqList[numScanRequests].addr, peerAddr, B_ADDR_LEN);

  // Convert address to string
  uint8_t *pAddr = (uint8_t*)Util_convertBdAddr2Str(peerAddr);

  // Copy converted string to static device list
  memcpy(scanReqList[numScanRequests].strAddr, pAddr, B_STR_ADDR_LEN);

  scanReqList[numScanRequests].bleChan = scanRequestReport->bleChan;
  scanReqList[numScanRequests].rssi = scanRequestReport->rssi;
  scanReqList[numScanRequests].addrType = scanRequestReport->peerAddrType;
  numScanRequests++;

#ifdef DEBUG_TSYNC
  Display_print2(dispHandle, 21, 0, "Num scan requests: %d, rssi: %d",
                                   numScanRequests, scanRequestReport->rssi);
#endif
}

//Add time Sync requests information to list
static void add_to_timeSync_reqList(gapDeviceInfoEvent_t deviceInfo)
{
  if(numTimeSyncRequests == DEFAULT_MAX_SCAN_REQ)
    return;
  uint8_t *peerAddr = deviceInfo.addr;
  for(int i = 0; i < numTimeSyncRequests; i++){
    if(memcmp(peerAddr, &(timeSyncReqList[i].addr), B_ADDR_LEN)== 0){
      //Device already exists in the list
      return;
    }
  }

  //Store time sync requester information
  memcpy(timeSyncReqList[numTimeSyncRequests].addr, peerAddr, B_ADDR_LEN);

  // Convert address to string
  uint8_t *pAddr = (uint8_t*)Util_convertBdAddr2Str(peerAddr);

  // Copy converted string to static device list
  memcpy(timeSyncReqList[numTimeSyncRequests].strAddr, pAddr, B_STR_ADDR_LEN);

  timeSyncReqList[numTimeSyncRequests].rssi = deviceInfo.rssi;
  timeSyncReqList[numTimeSyncRequests].addrType = deviceInfo.addrType;
  numTimeSyncRequests++;

#ifdef DEBUG_TSYNC
  Display_print3(dispHandle, 21, 0, "Num time sync requests: %d, rssi: %d, count: %d",
                 numTimeSyncRequests, deviceInfo.rssi, count++);
#endif
}

//Find the target address to direct advertise to
//Currently the algorithm for this is maximum rssi
static uint8_t *find_target_addr(void)
{
  if(numScanRequests < 1)
    return NULL;

  int8_t max_rssi = scanReqList[0].rssi;
  uint8_t *target_addr = scanReqList[0].addr;
  for(int i = 1; i < numScanRequests; i++){
    if(scanReqList[i].rssi > max_rssi){
      max_rssi = scanReqList[i].rssi;
      target_addr = scanReqList[i].addr;
    }
  }

  return target_addr;
}

//Check if data packet is for time synchronization
static bool is_timeSync_req(uint8_t * advData)
{
  return (advData[MSG_TYPE_POS] == TIME_SYNC_START);
}

//Check if data packet contains connect target address
static bool is_dirConnAddr_req(uint8_t * advData, int msg)
{
  return ((advData[MSG_TYPE_POS] == msg)
          && (memcmp(&advData[ADDR_POS], myAddr, B_ADDR_LEN) == 0));
}

//Check if time sync information has been saved
static bool saved_time_sync_info(uint8_t *addr)
{
  for(int i =0; i < numTimeSyncRequests; i++){
    if(memcmp(timeSyncReqList[i].addr, addr, B_ADDR_LEN) == 0)
      return TRUE;
  }

  return FALSE;
}

//Reset the saved results from scan discovery
static void reset_scan_discovery_info(void)
{
  memset(devList, 0, numScanDevs * sizeof(mrDevRec_t));
  numScanDevs = 0;
}

/*********************************************************************
*********************************************************************/
