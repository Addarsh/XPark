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

#define GATEWAY

#ifdef GATEWAY
  #define DEV_NUM 1
#else
  #define DEV_NUM 2
#endif

//#define DEBUG_TSYNC   //Debug printfs for time syncing
//#define DEBUG_PAIRING //Debug printfs for pairing

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

//Connecting timeout
#define CONN_TIMEOUT                         40000

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define SCAN_MODULO                           40000 //change: 4/6 s
#define SLEEPING_TIMEOUT                      20000
#define SCAN_REQ_DELAY_PERIOD                 20000 // change:2/4s to wait for scan responses
#define DEFAULT_SCAN_DURATION                 2000 //change: 2/4s
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

//Maximum number of scan requests possible
#define DEFAULT_MAX_SCAN_REQ                  10

//Maximum number of Time sync requests possible
#define DEFAULT_MAX_TSYNC_REQ                 10

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
#define MY_MANUFACTURER_ID_1                  0x1633
#define MY_MANUFACTURER_ID_2                  0x1634

//Time sync start message in advert data
#define DEFFAULT_NO_MESSAGE                    0xFF
#define TIME_SYNC_START                        42
#define DIRECT_ADV_CONN                        43
#define SPOT_UPDATE                            44

//Index for advert data positions
#define MSG_TYPE_POS                           11
#define ADDR_POS                               16

#define RANDOM_DELAY                          50000  //5s

//Time for sensor to be awake
#define AWAKE_TIME                            20000   //2s

//Sensor absolute waking time
#define GLOBAL_WAKEUP_TIME                    200000  //20s

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
#define GO_TO_SLEEP_EVT                      Event_Id_08
#define WRITE_DATA                           Event_Id_09
#define MASTER_WRITE_RECVD                   Event_Id_10
#define START_SCAN                           Event_Id_11
#define CONN_TIMEOUT_EVT                     Event_Id_12
#define SCAN_REQ_TIMEOUT                     Event_Id_13
#define DIR_ADV_TIMEOUT                      Event_Id_14
#define START_DIR_ADV                        Event_Id_15
#define STATUS_CHANGE_EVT                    Event_Id_16
#define END_ROUTE_SCAN                       Event_Id_17
#define DISCOVERABLE_NOW                     Event_Id_18
#define NOT_DISCOVERABLE                     Event_Id_19
#define CONNECTION_COMPLETE                  Event_Id_20
#define DISCONNECTED                         Event_Id_21
#define TIMEOUT_WAIT_FOR_CONN                Event_Id_22
#define OOB_BONDING_COMPLETE                 Event_Id_23
#define OOB_BONDING_FAILED                   Event_Id_24
#define ENTERING_STATE                       Event_Id_25
#define TSYNC_MASTER_INIT                    Event_Id_26
#define TCHANGE_INIT                         Event_Id_27
#define CC_ENABLED                           Event_Id_28
#define SLEEP_WAKEUP_EVT                     Event_Id_29

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
                                             GO_TO_SLEEP_EVT         | \
                                             WRITE_DATA              | \
                                             MASTER_WRITE_RECVD      | \
                                             START_SCAN              | \
                                             CONN_TIMEOUT_EVT        | \
                                             SCAN_REQ_TIMEOUT        | \
                                             DIR_ADV_TIMEOUT         | \
                                             START_DIR_ADV           | \
                                             STATUS_CHANGE_EVT       | \
                                             END_ROUTE_SCAN          | \
                                             DISCOVERABLE_NOW        | \
                                             NOT_DISCOVERABLE        | \
                                             CONNECTION_COMPLETE     | \
                                             DISCONNECTED            | \
                                             TIMEOUT_WAIT_FOR_CONN   | \
                                             OOB_BONDING_COMPLETE    | \
                                             OOB_BONDING_FAILED      | \
                                             ENTERING_STATE          | \
                                             TSYNC_MASTER_INIT       | \
                                             TCHANGE_INIT            | \
                                             CC_ENABLED              | \
                                             SLEEP_WAKEUP_EVT)

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
#define DIR_ADV_PERIOD                       40000 //5s wait time for dir advertising
#define WAIT_FOR_CONN_DURATION               60000 //6s to wait to connect

//Invalid node info
#define INVALID_NODE_INFO -1
#define VALID_NODE_INFO 16

//Route direc adv max count
#define ROUTE_DIRECT_ADV_MAX_COUNT        1

//Maximum interval for random route discovery
#ifdef GATEWAY
    #define ROUTE_DISC_WAIT_MAX_CNT          3
    #define ROUTE_DISC_WAIT_MIN_CNT          1
#else
    #define ROUTE_DISC_WAIT_MAX_CNT          10
    #define ROUTE_DISC_WAIT_MIN_CNT          8
#endif

//OOB key len
#define OOB_KEY_LEN                         16

//Alternate names for RED and GREEN leds
#define RED Board_RLED
#define GREEN Board_GLED

//Invalid BLE connection handle
#define INVALID_CONN_HANDLE            UINT16_MAX

//Family Filter length is 5 bytes
#define FAMILY_FILTER_LEN                   5
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
static Clock_Struct routeScanClock;
static Clock_Struct waitForConnClock;
static Clock_Struct connTimeoutClock;
static Clock_Struct randomClock;
static Clock_Struct sleepWakeUpClock;
static Clock_Struct awakeClock;

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
  TSYNCED_SLEEPING,
  TSYNCED_SCANNING,
  TSYNCED_MASTER,
  USYNCED_ADV,
  USYNCED_SLAVE,
  TSYNCED_CHANGE_SLAVE,
  TSYNCED_CHANGE_MASTER,
};

//Time synced master (TSM) states
enum tsynced_master_t{
  TSM_CONNECTING,
  TSM_PAIRING,
  TSM_PAIRED,
  TSM_CC_ENABLED,
  TSM_T1_SENT,
  TSM_DELAY_RSP_SENT
};

//Unsynced slave states
enum usyncslave_t{
  USYNCSLAVE_PAIRING,
  USYNCSLAVE_PAIRED,
  USYNCSLAVE_WRITE_MASTER,
  USYNCSLAVE_T2_PRIME_SENT
};

//Unsynced advertising states
enum usyncadv_t{
  USYNCADV_PASSIVE,
  USYNCADV_DIRECT,
  USYNCADV_CONNECTING
};

//Tsynced change slave advertising
enum tchange_slave_t{
  TCHANGE_SLAVE_ADV,
  TCHANGE_SLAVE_BONDING_CNF,
  TCHANGE_SLAVE_BONDED,
  TCHANGE_SLAVE_SENT_DATA
};

//Tsynced change master
enum tchange_master_t{
  TCHANGE_MASTER_CONNECTING,
  TCHANGE_MASTER_CONNECTED,
  TCHANGE_MASTER_BONDED,
  TCHANGE_MASTER_WAITING
};

static const uint8_t modified_addr[B_ADDR_LEN] = {DEV_NUM, 0x4, 0x8, 0x9, 0x0, 0xB};

static uint8_t OOB_key[OOB_KEY_LEN] = {0x2b,0x7e,0x15,0x16,
                                       0x28,0xae,0xd2,0xa6,
                                       0xab,0xf7,0x15,0x88,
                                       0x09,0xcf,0x4f,0x3c};
static uint8_t family_filter[FAMILY_FILTER_LEN] = {0x4, 0x8, 0x9, 0x0, 0xB};

//Global state var
static int global_state;

//Function pointer declarations for entry and exit functions
static void (*entry_func)(void *);
static void (*exit_func)(void);

//Global clock for time syncronization
static uint64_t my_global_time;

//Last updated parking status
static uint64_t last_spot_status = 2;
static uint64_t curr_spot_status;

//Current connection handle
static uint16_t curr_conn_handle = INVALID_CONN_HANDLE;
static uint8_t curr_peer_addr_type;
static uint8_t curr_peer_addr[B_ADDR_LEN];

//Time variables used for synchronization of slave
static int64_t T1;
static int64_t T1_prime;
static int64_t T2;
static int64_t T2_prime;

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
static void routeScan_clockHandler(UArg arg);
static void  waitForConn_clockHandler(UArg arg);
static void connTimeout_clockHandler(UArg arg);
static void random_clockHandler(UArg arg);
static void awake_clockHandler(UArg arg);
static void wakeup_clockHandler(UArg arg);

static bool enable_notifs(uint8_t index);
static uint64_t get_my_global_time(void);
static bool write_to_server(uint8_t index, uint64_t time);
static void perform_time_sync(void);

//Common functions
static void enter_state(enum global_state_t new_state);
static int set_and_start_advertising(uint8_t advType,
                                      uint8_t advdata_msg,
                                      uint8_t * target_addr);
static int stop_advertising(void);
static int start_scanning(void);
static int stop_scanning(void);
static void turn_on_led(int ledpin);
static void turn_off_led(int ledpin);
static int save_scan_device_info(uint8_t addrType, uint8_t *addr);

//TSCAN related functions
static void tscan_processEvts(uint32_t events);

//TSLEEP related functions
static void tsleep_processEvts(uint32_t events);

//TCHANGE Slave related functions
static void tchange_slave_entry_func(void *sub_state);
static void tchange_slave_exit_func(void);
static void tchange_slave_processEvts(uint32_t events);

//TCHANGE Master related functions
static void tchange_master_entry_func(void *sub_state);
static void tchange_master_exit_func(void);
static void tchange_master_processEvts(uint32_t events);

//TSM related functions
static void tsm_entry_func(void *sub_state);
static void tsm_exit_func(void);
static int tsm_write_data(uint64_t T);
static void tsm_processEvts(uint32_t events);

//USYNCADV related functions
static void usyncadv_entry_func(void *sub_state);
static void usyncadv_exit_func(void);
static void save_next_hop_info(uint8_t *addr);
static void usyncadv_processEvts(uint32_t events);

//USYNCSLAVE related functions
static void usyncslave_entry_func(void *sub_state);
static void usyncslave_exit_func(void);
static void bad_exit(void);
static void usyncslave_processEvts(uint32_t events);

static void addToScanReqReceivedList(hciEvt_BLEScanReqReport_t* scanRequestReport);
static uint8_t is_family(uint8_t *addr);
static uint8_t *find_target_addr(int8_t *rssi);
static bool cancel_connect(void);
static bool connect(uint8_t addrType, uint8_t* addr);
static bool is_timeSync_req(uint8_t * advData);
static bool is_dirConnAddr_req(uint8_t * advData, int  msg);
static void add_to_timeSync_reqList(gapDeviceInfoEvent_t timeSyncSlave);
static bool saved_time_sync_info(uint8_t *addr);
static void reset_scan_discovery_info(void);
static uint64_t get_spot_status(void);

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

  //Set bluetooth address here
  HCI_EXT_SetBDADDRCmd(modified_addr);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, global_time_clockHandler,
                      GLOBAL_TIME_CLOCK_PERIOD, GLOBAL_TIME_CLOCK_PERIOD, false, 0);
  Util_constructClock(&passiveAdvertClock, scanRequest_timeoutHandler,
                             SCAN_REQ_DELAY_PERIOD, 0, false, SCAN_REQ_TIMEOUT);
  Util_constructClock(&dirAdvClock, dirAdv_clockHandler,
                        DIR_ADV_PERIOD, 0, false, DIR_ADV_TIMEOUT);
  Util_constructClock(&routeScanClock, routeScan_clockHandler,
                      DEFAULT_SCAN_DURATION, 0, false, END_ROUTE_SCAN);
  Util_constructClock(&waitForConnClock, waitForConn_clockHandler,
                        WAIT_FOR_CONN_DURATION, 0, false, TIMEOUT_WAIT_FOR_CONN);
  Util_constructClock(&connTimeoutClock, connTimeout_clockHandler,
                            CONN_TIMEOUT, 0, false, CONN_TIMEOUT_EVT);
  Util_constructClock(&randomClock, random_clockHandler,
                              RANDOM_DELAY, 0, false, OOB_BONDING_COMPLETE);
  Util_constructClock(&awakeClock, awake_clockHandler,
                                AWAKE_TIME, 0, false, GO_TO_SLEEP_EVT);

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

  //Tx power to 2 dbm
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_4_DBM);

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
    uint8_t sc_mode = GAPBOND_SECURE_CONNECTION_NONE;
    uint8_t oob_enabled = TRUE;

    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t bonding = TRUE;

    // Set pairing mode
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);

    // Set authentication requirements
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);

    // Set I/O capabilities
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);

    // Sst bonding requirements
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);


    //Set Secure connections (for now, change later when you understand)
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &sc_mode);

    //Set OOB enable
    GAPBondMgr_SetParameter(GAPBOND_OOB_ENABLED, sizeof(uint8_t), &oob_enabled);

    //Set OOB data
    GAPBondMgr_SetParameter(GAPBOND_OOB_DATA, KEYLEN, OOB_key);

    //Erase all bonds on reset
    GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS,  0, NULL);

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

      if(events & START_SCAN)
      {
        uint64_t my_time = get_my_global_time();
        Display_print1(dispHandle,12,0,"p: %d", my_time);
      }

       //Manage global application states here
      switch(global_state){
        case TSYNCED_SCANNING:
          tscan_processEvts(events);
        break;

        case TSYNCED_SLEEPING:
          tsleep_processEvts(events);
        break;

        case TSYNCED_CHANGE_SLAVE:
          tchange_slave_processEvts(events);
        break;

        case TSYNCED_CHANGE_MASTER:
          tchange_master_processEvts(events);
        break;

        case USYNCED_ADV:
          usyncadv_processEvts(events);
        break;

        case USYNCED_SLAVE:
          usyncslave_processEvts(events);
        break;

        case TSYNCED_MASTER:
          tsm_processEvts(events);
        break;

        default:
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
//#ifdef DEBUG_TSYNC
    Display_print1(dispHandle, 12, 0, "T2: %u", T2);
//#endif

    Event_post(syncEvent, WRITE_DATA);
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

      //Initialize starting state
    #ifdef GATEWAY
      enter_state(TSYNCED_SCANNING);
    #else
      enter_state(USYNCED_ADV);
    #endif

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
      //Check if directed advertisement, if so, connect immediately
      if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND &&
             saved_time_sync_info(pEvent->deviceInfo.addr)
            && is_dirConnAddr_req(pEvent->deviceInfo.pEvtData, DIRECT_ADV_CONN))
      {
        if(save_scan_device_info(pEvent->deviceInfo.addrType, pEvent->deviceInfo.addr) == SUCCESS)
          Event_post(syncEvent, TSYNC_MASTER_INIT);
      }
      else if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND &&
              is_dirConnAddr_req(pEvent->deviceInfo.pEvtData, SPOT_UPDATE))
      {
        if(save_scan_device_info(pEvent->deviceInfo.addrType, pEvent->deviceInfo.addr) == SUCCESS)
          Event_post(syncEvent, TCHANGE_INIT);
      }
      else if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_SCAN_IND  &&
              is_timeSync_req(pEvent->deviceInfo.pEvtData))
      {
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
      turn_off_led(RED);

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

        curr_conn_handle = pEvent->linkCmpl.connectionHandle;
        if(global_state == USYNCED_ADV)
        {
          save_next_hop_info(pEvent->linkCmpl.devAddr);
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
      Event_post(syncEvent, MASTER_WRITE_RECVD);
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
      Event_post(syncEvent, CC_ENABLED);
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

        if(global_state == TSYNCED_MASTER || global_state == TSYNCED_CHANGE_MASTER)
        {
          Event_post(syncEvent, NOTIFY_ENABLE);
        }

    #ifdef DEBUG_TSYNC
        Display_print0(dispHandle,6,0,"recieved all characteristics!");
    #endif
      }else
      {
    #ifdef DEBUG_TSYNC
          Display_print0(dispHandle,6,0,"recieved nothing!");
    #endif
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
      Event_post(syncEvent, OOB_BONDING_COMPLETE);
    }
  }
  // If a bond has been saved
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      Display_print1(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
      Util_startClock(&randomClock);
    }
    else
    {
      Display_print2(dispHandle, MR_ROW_SECURITY, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
      Event_post(syncEvent, OOB_BONDING_FAILED);
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
  if(my_global_time % SCAN_MODULO == 0){
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
 * @fn      connTimeout_clockHandler
 *
 * @brief   Connection timeout handler
 *
 * @param   arg - event type
 */
static void connTimeout_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}


/*********************************************************************
 * @fn      random_clockHandler
 *
 * @brief   random timeout handler
 *
 * @param   arg - event type
 */
static void random_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      awake_clockHandler
 *
 * @brief   awake clock handler
 *
 * @param   arg - event type
 */
static void awake_clockHandler(UArg arg)
{
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      wakeup_clockHandler
 *
 * @brief   wakeup clock handler
 *
 * @param   arg - event type
 */
static void wakeup_clockHandler(UArg arg)
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

//Cancel given connection
static bool cancel_connect(void)
{
  // Cancel connection request
  GAPRole_TerminateConnection(GAP_CONNHANDLE_INIT);
  Display_print0(dispHandle, MR_ROW_STATUS1, 0, "Connecting Cancelled");

  // Clear connecting flag
  connecting = FALSE;

  return TRUE;
}

//Connect to given public address with given addrType
static bool connect(uint8_t addrType, uint8_t* addr)
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

/*-----------COMMON FUNCTIONS------------------------------------------------*/

//Enter the given state
static void enter_state(enum global_state_t new_state)
{
  global_state = new_state;
  switch(new_state)
  {
    case TSYNCED_SCANNING:
    case TSYNCED_SLEEPING:
      entry_func = NULL;
      exit_func = NULL;
    break;

    case TSYNCED_MASTER:
      entry_func = tsm_entry_func;
      exit_func = tsm_exit_func;
    break;

    case USYNCED_ADV:
      entry_func = usyncadv_entry_func;
      exit_func = usyncadv_exit_func;
    break;

    case USYNCED_SLAVE:
      entry_func = usyncslave_entry_func;
      exit_func = usyncslave_exit_func;
    break;

    case TSYNCED_CHANGE_SLAVE:
      entry_func = tchange_slave_entry_func;
      exit_func = tchange_slave_exit_func;
    break;

    case TSYNCED_CHANGE_MASTER:
      entry_func = tchange_master_entry_func;
      exit_func = tchange_master_exit_func;
    break;
  }

  Event_post(syncEvent, ENTERING_STATE);
}

//Set advertising parameters and start advertising
//Advertising paramters passed may change in the futures
static int set_and_start_advertising(uint8_t advType, uint8_t advdata_msg, uint8_t * target_addr)
{
  //Get current advertising status
  uint8_t adv_status;
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);

  if(adv_status)
  {
    Display_print0(dispHandle, 20, 0, "Error! Device is already advertising!");
    return FAILURE;
  }

  GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType, NULL);

  //Set advertising to time-sync
  advertData[MSG_TYPE_POS] = advdata_msg;

  if(target_addr != NULL)
    memcpy(&advertData[ADDR_POS], target_addr, B_ADDR_LEN);

  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

  //Turn on advertising
  mr_doAdvertise(0);

  return SUCCESS;
}

//Stop advertising
static int stop_advertising(void)
{
  //Get current advertising status
  uint8_t adv_status;
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &adv_status, NULL);

  if(!adv_status)
  {
    Display_print0(dispHandle, 20, 0, "Error! Device is already NOT advertising!");
    return FAILURE;
  }

  mr_doAdvertise(0);

  return SUCCESS;
}

//Start scanning
static int start_scanning(void)
{
  if(!scanningStarted)
  {
    mr_doScan(0);
    return SUCCESS;
  }
  return FAILURE;
}

//Stop scanning
static int stop_scanning(void)
{
  if(scanningStarted)
  {
    mr_doScan(0);
    return SUCCESS;
  }
  return FAILURE;
}

//Turn on LED
static void turn_on_led(int ledpin)
{
  //Turn on Red LED
  switch(ledpin)
  {
    case RED:
        PIN_setOutputValue(ledPinHandle, Board_RLED, 1);
        break;

    case GREEN:
        PIN_setOutputValue(ledPinHandle, Board_GLED, 1);
        break;

    default:
        break;
  }
}

//Turn off LED
static void turn_off_led(int ledpin)
{
  //Turn on Red LED
  switch(ledpin)
  {
    case RED:
        PIN_setOutputValue(ledPinHandle, Board_RLED, 0);
        break;

    case GREEN:
        PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
        break;

    default:
        break;
  }
}

//Saving scanned peer device information
static int save_scan_device_info(uint8_t addrType, uint8_t *addr)
{
  //Stop scanning now
  if(stop_scanning() == FAILURE)
  {
    Display_print0(dispHandle, 20, 0, "Error! Device is not scanning! Can't stop!");
    return FAILURE;
  }
  turn_off_led(RED);

  //Save peer device details
  curr_peer_addr_type = addrType;
  memcpy(curr_peer_addr, addr, B_ADDR_LEN);

  return SUCCESS;
}

/*-----------------------------------------------------------------------*/

/*-------------------TSYNCED_SCANNING---------------------------------------*/

//Handle Tsynced scanning events
static void tscan_processEvts(uint32_t events)
{
  if(events & START_SCAN)
  {
    if(start_scanning() == FAILURE){
      Display_print0(dispHandle, 20, 0, "Error!!Already scanning!");
    }
    else
    {
      turn_on_led(RED);
    }
  }
  else if(events & TSYNC_MASTER_INIT)
  {
    //Switch to master state
    enter_state(TSYNCED_MASTER);

    connect(curr_peer_addr_type, curr_peer_addr);

    //Connecting timeout clock
    Util_startClock(&connTimeoutClock);

  #ifdef DEBUG_TSYNC
    Display_print1(dispHandle,18,0,"Tsync master connecting to: %s",
                     (const char*)Util_convertBdAddr2Str(curr_peer_addr));
  #endif

  }
  else if(events & TCHANGE_INIT)
  {
    //Switch to tchange master state
    enter_state(TSYNCED_CHANGE_MASTER);

    connect(curr_peer_addr_type, curr_peer_addr);

    //Connecting timeout clock
    Util_startClock(&connTimeoutClock);
  }
}

/*-----------------------------------------------------------------------*/

/*-------------------TSYNCED_SLEEPING---------------------------------------*/

//Handle Tsynced sleeping events
static void tsleep_processEvts(uint32_t events)
{
  static int count = 0;
  if(events & ENTERING_STATE)
  {
   if(count == 0)
   {
     Util_constructClock(&sleepWakeUpClock, wakeup_clockHandler,
                              GLOBAL_WAKEUP_TIME - (get_my_global_time() % GLOBAL_WAKEUP_TIME),
                             0, true, SLEEP_WAKEUP_EVT);
     count++;
   }
   else
   {
     Util_restartClock(&sleepWakeUpClock, GLOBAL_WAKEUP_TIME - (get_my_global_time() % GLOBAL_WAKEUP_TIME));
   }
  }
  else if(events & SLEEP_WAKEUP_EVT)
  {
    Display_print0(dispHandle, 18, 0, "waking up!");
    curr_spot_status = get_spot_status();

    if(curr_spot_status != last_spot_status)
    {
      turn_on_led(GREEN);
      Util_startClock(&awakeClock);
      enter_state(TSYNCED_CHANGE_SLAVE);
    }else
    {
      Display_print0(dispHandle, 18, 0, "entering sleep!");
      enter_state(TSYNCED_SLEEPING);
    }
  }
  else if(events & GO_TO_SLEEP_EVT)
  {
    turn_off_led(GREEN);

    enter_state(TSYNCED_SLEEPING);
  }
}

/*-----------------------------------------------------------------------*/

/*-------------------TCHANGE_SLAVE---------------------------------------*/
//Entry function
static void tchange_slave_entry_func(void *sub_state)
{
  if(set_and_start_advertising(GAP_ADTYPE_ADV_IND,
                               SPOT_UPDATE, next_hop_node.addr) == FAILURE)
    return;

  *(enum tchange_slave_t *)sub_state = TCHANGE_SLAVE_ADV;
}

//Exit function
static void tchange_slave_exit_func(void)
{
  turn_off_led(GREEN);

  //Reset scanReqList
  memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
  numScanRequests = 0;

  //Reset connection handler
  curr_conn_handle = INVALID_CONN_HANDLE;

  Display_print0(dispHandle, 18, 0, "entering sleep!");
  enter_state(TSYNCED_SLEEPING);
}

//Handle Tchange status change events
static void tchange_slave_processEvts(uint32_t events)
{
  static enum tchange_slave_t tchange_slave_state;

  if(events & ENTERING_STATE)
  {
    entry_func(&tchange_slave_state);
    return;
  }

  if(events & DISCONNECTED)
  {
    exit_func();
    return;
  }

  switch(tchange_slave_state)
  {
    case TCHANGE_SLAVE_ADV:
      if(events & GO_TO_SLEEP_EVT)
      {
        stop_advertising();
        exit_func();
      }
      else if(events & CONNECTION_COMPLETE)
      {
        tchange_slave_state = TCHANGE_SLAVE_BONDING_CNF;
        stop_advertising();
      }
   break;

    case TCHANGE_SLAVE_BONDING_CNF:
      if(events & OOB_BONDING_COMPLETE)
      {
        tchange_slave_state = TCHANGE_SLAVE_BONDED;
      }
      else if(events & OOB_BONDING_FAILED)
      {
        Display_print0(dispHandle, 17, 0,"Bonding failed with master!");
        exit_func();
      }
    break;

    case TCHANGE_SLAVE_BONDED:
      if(events & CC_ENABLED)
      {
        //Send  status
        if(SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
                                      SIMPLEPROFILE_CHAR4_LEN, &curr_spot_status) == FAILURE)
        {
          Display_print0(dispHandle, 15, 0, "Error! Notification send failed!");
          Event_post(syncEvent, CC_ENABLED);
        }
        else
        {
          tchange_slave_state = TCHANGE_SLAVE_SENT_DATA;
          last_spot_status =  curr_spot_status;
        }
      }
    break;

    case TCHANGE_SLAVE_SENT_DATA:
      //Nothing to be done for now
    break;
  }
}

/*-----------------------------------------------------------------------*/

/*-------------------TCHANGE_MASTER---------------------------------------*/
//Entry function
static void tchange_master_entry_func(void *sub_state)
{
  *(enum tchange_master_t *)sub_state = TCHANGE_MASTER_CONNECTING;
}

//Exit function
static void tchange_master_exit_func(void)
{
  //Reset time sync req list
  memset(timeSyncReqList, 0, numTimeSyncRequests*sizeof(mrDevRec_t));
  numTimeSyncRequests = 0;

  //Reset connection handler
  curr_conn_handle = INVALID_CONN_HANDLE;

  enter_state(TSYNCED_SCANNING);
}

//Handle Tchange master status change events
static void tchange_master_processEvts(uint32_t events)
{
  static enum tchange_master_t tchange_master_state;

  if(events & ENTERING_STATE)
  {
    entry_func(&tchange_master_state);
    return;
  }

  if(events & DISCONNECTED)
  {
    exit_func();
    return;
  }

  switch(tchange_master_state)
  {
    case TCHANGE_MASTER_CONNECTING:
      if(events & CONN_TIMEOUT_EVT)
      {
        cancel_connect();
        exit_func();
      }
      else if(events & CONNECTION_COMPLETE)
      {
        tchange_master_state = TCHANGE_MASTER_CONNECTED;
      }
    break;

    case TCHANGE_MASTER_CONNECTED:
      if(events & OOB_BONDING_COMPLETE)
      {
        tchange_master_state = TCHANGE_MASTER_BONDED;

        //Start service discovery
        multi_role_startDiscovery(curr_conn_handle);
      }
      else if(events & OOB_BONDING_FAILED)
      {
        exit_func();
      }
    break;

    case TCHANGE_MASTER_BONDED:
      if(events & NOTIFY_ENABLE)
      {
        //Enable CC in client
        if(enable_notifs(0) == FAILURE)
        {
          Event_post(syncEvent, NOTIFY_ENABLE);
        }
        else
        {
          tchange_master_state = TCHANGE_MASTER_WAITING;
        }
      }
    break;

    case TCHANGE_MASTER_WAITING:
      if(events & WRITE_DATA)
      {
        //Received spot data from slave
        //T2 Variable contains the spot update
        if(T2 == 0)
        {
          Display_print0(dispHandle, 20, 0, " Spot is open!");
        }
        else
        {
          Display_print0(dispHandle, 20, 0, "Spot is occupied!");
        }

        mr_doDisconnect(0);

        exit_func();
      }
    break;
  }
}

/*-----------------------------------------------------------------------*/

/*-------------------TSYNCED_MASTER---------------------------------------*/

//Entry function for TSM
static void tsm_entry_func(void *sub_state)
{
  *(enum tsynced_master_t *)sub_state = TSM_CONNECTING;
}

//Exit function for TSM
static void tsm_exit_func(void)
{
  //Reset time sync req list
  memset(timeSyncReqList, 0, numTimeSyncRequests*sizeof(mrDevRec_t));
  numTimeSyncRequests = 0;

  //Reset connection handler
  curr_conn_handle = INVALID_CONN_HANDLE;

  //Go back to tsycned scanning
  enter_state(TSYNCED_SCANNING);
}

//Data written by master to slave in time sync
static int tsm_write_data(uint64_t T)
{
  if(write_to_server(0,T) == FAILURE)
  {
 #ifdef DEBUG_TSYNC
    Display_print0(dispHandle,15,0,"Error! Write to server failed!");
 #endif
    Event_post(syncEvent, WRITE_DATA);
    return FAILURE;
  }
  return SUCCESS;
}

//Handle Master events related to time synchronization
static void tsm_processEvts(uint32_t events)
{
  static enum tsynced_master_t tsm_state;

  if(events & ENTERING_STATE)
  {
    entry_func(&tsm_state);
    return;
  }

  if(events & DISCONNECTED)
  {
    exit_func();
    return;
  }

  switch (tsm_state)
  {
    case TSM_CONNECTING:
      if(events & CONN_TIMEOUT_EVT)
      {
        cancel_connect();
        exit_func();
      }
      else if(events & CONNECTION_COMPLETE)
      {
        tsm_state = TSM_PAIRING;
      }
    break;

    case TSM_PAIRING:
      if(events & OOB_BONDING_COMPLETE)
      {
    #ifdef DEBUG_PAIRING
        Display_print0(dispHandle, 16, 0, "Pairing complete!");
    #endif
        //Start service discovery
        multi_role_startDiscovery(curr_conn_handle);

        tsm_state = TSM_PAIRED;
      }
      else if(events & OOB_BONDING_FAILED)
      {
    #ifdef DEBUG_PAIRING
        Display_print0(dispHandle, 16, 0, "Pairing failed! Disconnect!");
    #endif
        //Disconnect
        mr_doDisconnect(0);

        //Exit state
        exit_func();
      }
    break;

    case TSM_PAIRED:
      if (events & NOTIFY_ENABLE)
      {
        //Enable CC in client
        if(enable_notifs(0) == FAILURE)
        {
          Event_post(syncEvent, NOTIFY_ENABLE);
        }
        else
        {
          tsm_state = TSM_CC_ENABLED;
          Event_post(syncEvent, WRITE_DATA);
        }
      }
    break;

    case TSM_CC_ENABLED:
      if(events & WRITE_DATA)
      {
        //Notify the client, call SetParameter
        T1 = get_my_global_time();
        if(tsm_write_data(T1) == SUCCESS)
          tsm_state = TSM_T1_SENT;
        else
          Event_post(syncEvent, WRITE_DATA);
      }
    break;

    case TSM_T1_SENT:
      if(events & WRITE_DATA)
      {
        //Construct delay response message
        T2_prime = get_my_global_time();
        if(tsm_write_data(T2_prime) == SUCCESS)
          tsm_state = TSM_DELAY_RSP_SENT;
        else
          Event_post(syncEvent, WRITE_DATA);
      }
    break;

    case TSM_DELAY_RSP_SENT:
       //For now, nothing to do here
     break;

    default:
    break;
  }
}
/*--------------------------------------------------------------------*/

/*---------------------UNSYNCED_ADVERTISING----------------------------*/
static void usyncadv_entry_func(void *sub_state)
{
  *(enum usyncadv_t *)(sub_state) = USYNCADV_PASSIVE;

  //Reset scanReqList
  memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
  numScanRequests = 0;

  //Set advertising type to scannable, non-connectable indirect advertising
  if(set_and_start_advertising(GAP_ADTYPE_ADV_SCAN_IND,
                               TIME_SYNC_START, NULL) == FAILURE)
    return;

  turn_on_led(RED);
}

//Save node information of current peer as next hop node
static void save_next_hop_info(uint8_t *addr)
{
  memcpy(next_hop_node.addr, addr, B_ADDR_LEN);
  next_hop_node.valid = VALID_NODE_INFO;

  Display_print2(dispHandle, 16, 0, "Next hop addr: %s, rssi: %d",
                 (const char*)Util_convertBdAddr2Str(next_hop_node.addr),
                 next_hop_node.rssi);
}

static void usyncadv_exit_func(void)
{
  stop_advertising();

  //Next state is USYNC_SLAVE
  enter_state(USYNCED_SLAVE);
}

static void usyncadv_processEvts(uint32_t events)
{
  static enum usyncadv_t usyncadv_state;

  if(events & ENTERING_STATE)
  {
    entry_func(&usyncadv_state);
    return;
  }

  switch(usyncadv_state)
  {
    case USYNCADV_PASSIVE:
      if(events & DISCOVERABLE_NOW)
      {
      #ifdef DEBUG_TSYNC
        Display_print0(dispHandle, 15, 0, "starting passive clock!");
      #endif
        //Start clock for passive advertising
        Util_startClock(&passiveAdvertClock);
      }
      else if(events & SCAN_REQ_TIMEOUT)
      {
        if(stop_advertising() == FAILURE)
          return;

      }
      else if(events & NOT_DISCOVERABLE)
      {
        usyncadv_state = USYNCADV_DIRECT;
        Event_post(syncEvent, START_DIR_ADV);
      }
    break;

    case USYNCADV_DIRECT:
      if(events & START_DIR_ADV)
      {
        uint8_t *target_addr = find_target_addr(&next_hop_node.rssi);

        if(target_addr == NULL)
        {
        #ifdef DEBUG_TSYNC
          Display_print0(dispHandle, 18, 0, "No address found go back!");
        #endif
          //No address found, try again
          enter_state(USYNCED_ADV);
        }
        else
        {
          if(set_and_start_advertising(GAP_ADTYPE_ADV_IND,
                                       DIRECT_ADV_CONN, target_addr) == FAILURE)
            return;
      #ifdef DEBUG_TSYNC
          Display_print1(dispHandle,20,0,"direct advertising has started to: %s!",
                           (const char*)Util_convertBdAddr2Str(target_addr));
      #endif
        }
      }
      else if(events & DISCOVERABLE_NOW)
      {
        //Start direct advertisement clock for dir advertising
        Util_startClock(&dirAdvClock);

        usyncadv_state = USYNCADV_CONNECTING;
      }
    break;

    case USYNCADV_CONNECTING:
      if(events & DIR_ADV_TIMEOUT)
      {
        stop_advertising();
      }
      else if(events & NOT_DISCOVERABLE)
      {
        //Direct adv failed, try again
        enter_state(USYNCED_ADV);
      }
      else if(events & CONNECTION_COMPLETE)
      {
        //Exit state
        exit_func();
      }
    break;
  }
}
/*---------------------------------------------------------------*/

/*---------------------UNSYNCED_SLAVE----------------------------*/
static void usyncslave_entry_func(void *sub_state)
{
  *(enum usyncslave_t *)sub_state = USYNCSLAVE_PAIRING;
}

//Delete next node information
static void bad_exit(void)
{
  //Erase hop info
  memset(next_hop_node.addr, 0, 0);
  next_hop_node.rssi = 0;
  next_hop_node.valid = INVALID_NODE_INFO;

  turn_off_led(RED);

  //Reset scanReqList
  memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
  numScanRequests = 0;

  curr_conn_handle = INVALID_CONN_HANDLE;

  enter_state(USYNCED_ADV);
}

static void usyncslave_exit_func(void)
{
  mr_doDisconnect(0);

  turn_off_led(RED);

  curr_conn_handle = INVALID_CONN_HANDLE;

  //Reset scanReqList
  memset(scanReqList, 0, sizeof(mrDevRec_t) * DEFAULT_MAX_SCAN_REQ);
  numScanRequests = 0;

  enter_state(TSYNCED_SLEEPING);
}

//Handle Slave events related to time synchronization
static void usyncslave_processEvts(uint32_t events)
{
  static enum usyncslave_t usyncslave_state;

  if(events & DISCONNECTED)
  {
    bad_exit();
    return;
  }

  switch(usyncslave_state)
  {
    case USYNCSLAVE_PAIRING:
      if(events & OOB_BONDING_COMPLETE)
      {
        usyncslave_state = USYNCSLAVE_PAIRED;
      }
      else if(events & OOB_BONDING_FAILED)
      {
        Display_print0(dispHandle, 15, 0, "Pairing failed! Disconnect now!");

        mr_doDisconnect(0);

        bad_exit();
      }
    break;

    case USYNCSLAVE_PAIRED:
      if(events & MASTER_WRITE_RECVD)
      {
        //Store times T1 and T1_prime
        T1_prime = get_my_global_time();
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &T1);
      #ifdef DEBUG_TSYNC
        Display_print2(dispHandle, MR_ROW_STATUS2, 0, "t1: %u, t1_prime: %u", (uint32_t)T1, (uint32_t)T1_prime);
      #endif

        usyncslave_state = USYNCSLAVE_WRITE_MASTER;
        Event_post(syncEvent, MASTER_WRITE_RECVD);
      }
    break;

    case USYNCSLAVE_WRITE_MASTER:
      if(events & MASTER_WRITE_RECVD)
      {
        //Notify the master of time T2
        T2 = get_my_global_time();
        if(SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,
                                      SIMPLEPROFILE_CHAR4_LEN,&T2) == FAILURE)
        {
          Display_print0(dispHandle, 15, 0, "Error! Notification send failed!");
          Event_post(syncEvent, MASTER_WRITE_RECVD);
        }
        else
        {
          usyncslave_state = USYNCSLAVE_T2_PRIME_SENT;
        }
      }
    break;

    case USYNCSLAVE_T2_PRIME_SENT:
      if(events & MASTER_WRITE_RECVD)
      {
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &T2_prime);
      #ifdef DEBUG_TSYNC
        Display_print1(dispHandle, MR_ROW_STATUS2+2, 0, "t1 : %u", (uint32_t)T1);
        Display_print1(dispHandle, MR_ROW_STATUS2+3, 0, "t1_prime: %u", (uint32_t)T1_prime);
        Display_print1(dispHandle, MR_ROW_STATUS2+4, 0, "t2: %u", (uint32_t)T2);
        Display_print1(dispHandle, MR_ROW_STATUS2+5, 0, "t2_prime: %u", (uint32_t)T2_prime);
      #endif

        perform_time_sync();

        exit_func();
      }
    break;
  }
}
/*---------------------------------------------------------------*/

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

  static int count = 17;
  Display_print1(dispHandle, count++, 0,"Scanned addr: %s",
                 (const char *)Util_convertBdAddr2Str(peerAddr));

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
  Display_print2(dispHandle, 21, 0, "Num time sync requests: %d, rssi: %d",
                 numTimeSyncRequests, deviceInfo.rssi);
#endif
}

//Check if peer node is part of family
static uint8_t is_family(uint8_t *addr)
{
  if(memcmp(family_filter, &addr[1], FAMILY_FILTER_LEN) == 0)
    return TRUE;

  return FALSE;
}

//Find the target address to direct advertise
static uint8_t *find_target_addr(int8_t *rssi)
{
  if(numScanRequests < 1)
    return NULL;

  uint8_t *target_addr = NULL;
  int8_t min_rssi = 0;

  for(int i = 0; i < numScanRequests; i++){
    if(is_family(scanReqList[i].addr) && scanReqList[i].rssi < min_rssi){
      min_rssi = scanReqList[i].rssi;
      target_addr = scanReqList[i].addr;
    }
  }

  *rssi = min_rssi;
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

//Returns the current spot status
//TODO: link to sensor value
static uint64_t get_spot_status(void)
{
  return (HalTRNG_GetTRNG() % 2);
}

/*********************************************************************
*********************************************************************/
