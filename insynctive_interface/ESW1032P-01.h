/******************************************************************************
;       includes - USB Device Stack (CDC Serial)
;                - Ethernet TCP/IP using ENC28J60
;                - SemTech SX1231 Transceiver (433MHz Ecolink protocol)
;                - ZWave ZM3102 (ZWave Bridge)
;
;   ESW1032P-01
;
;   Filename: ESW1032P-01.h
;
;   This file contains typdefs, enums, defines and prototype declarations
;   for the ESW1032P-01 (Boomerang) firmware.
;*******************************************************************************/
#ifndef __ESW1032P__H__
#define __ESW1032P__H__

//#define SERIAL_DEBUG_ON 1//test

#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <syslog.h>
#include <stdbool.h>
#include <unistd.h>

//#define DEBUG_DIAGS 1
//#define RF_TX_TEST  1

typedef unsigned char           BYTE;    /* 8-bit unsigned  */
typedef unsigned short int      WORD;    /* 16-bit unsigned */
typedef unsigned long           DWORD;   /* 32-bit unsigned */
typedef bool                    BOOL;    /* Undefined size */
typedef unsigned long long      QWORD;   /* 64-bit unsigned */


#define INSYNCTIVE_BRIDGE_VERSION_MSB 0x0A
#define INSYNCTIVE_BRIDGE_VERSION_LSB 58
#define _HUB_VERSION "A58"

#define HW_JAN_2013     1

#define XS1231_FIFO_MODE            1
#define XS1231_ENABLE_MANCHESTER    1
#define ECOLINK_PROTOCOL            1

//#define CRC_REVERSED        1

#define STANDALONE_MODE     0xFF//0xfe

// Boomerang is using CoreTimer 100ms for tick
#define BOOM_TICK_MINUTE    (DWORD)((60*1000)/100)

#define ENCRYPTED_PACKET_LENGTH 7

/** D E F I N E S ************************************************************/

enum STATE_MACHINE_STATES
{
    STATE_MACHINE_POR,
    STATE_MACHINE_LEARN_HUB,
    STATE_MACHINE_LEARN_DEVICE,
    STATE_MACHINE_LEARN_STATUS_MODULE,
    STATE_MACHINE_CLEAR_MEMORY,
    STATE_MACHINE_FACTORY_DEFAULT,
    STATE_MACHINE_DIAG_DEVICE_TO_HUB,
    STATE_MACHINE_DIAG_HUB_TO_ZWAVE,
    STATE_MACHINE_FIRMWARE_UPGRADE,
    STATE_MACHINE_LEARN_WAIT,     // 9  // this state is for a 5 second delay when StandAlone->ZWave auto-learn
    STATE_MACHINE_LEARN_SUCCESS,
    STATE_MACHINE_UNLEARN_SUCCESS,  //11
    STATE_MACHINE_LEARN_FAIL,
    STATE_MACHINE_LEARN_ABORT,
    STATE_MACHINE_HUB_LEARN_TIMEOUT,
    STATE_MACHINE_IDLE                //15
};

enum DEVICE_TYPE
{
    DEVICE_TYPE_DOOR_WINDOW_SENSOR  = 1,
    DEVICE_TYPE_SHOCK_SENSOR        = 2,
    DEVICE_TYPE_TILT_SENSOR         = 3,
    DEVICE_TYPE_MOTION_SENSOR       = 4,
    DEVICE_TYPE_FLOOD_SENSOR        = 5,
    DEVICE_TYPE_KEY_FOB             = 6,
    DEVICE_TYPE_WALL_PLATE          = 7,
    DEVICE_TYPE_WINDOW_LATCH        = 8,
    DEVICE_TYPE_TEMPERATURE_SENSOR  = 9,
    DEVICE_TYPE_HUMIDITY_SENSOR     = 10,
    DEVICE_TYPE_SIREN               = 11,
    DEVICE_TYPE_CHIME               = 12,
    DEVICE_TYPE_DEADBOLT_SENSOR     = 13,
    //DEVICE_TYPE_LIGHT_SWITCH        = 13,
    DEVICE_TYPE_APPLIANCE_PLUG_IN   = 14,
    DEVICE_TYPE_APPLIANCE_HIGH_CURRENT = 15,
    DEVICE_TYPE_LAMP_DIMMER         = 16,
    //DEVICE_TYPE_APPLIANCE_HIGH_CURRENT_VARIABLE_SPEED = 17,
    DEVICE_TYPE_INSYNCTIVE_BLIND_REMOTE = 17,
    DEVICE_TYPE_THERMOSTAT          = 18,
    DEVICE_TYPE_WINDOW_SHADE        = 19,
    DEVICE_TYPE_SMOKE_SENSOR        = 20,
    DEVICE_TYPE_CO_DETECTOR         = 21,
    DEVICE_TYPE_LIGHT_SWITCH        = 22,
    DEVICE_TYPE_STATUS_MODULE       = 30,
    DEVICE_TYPE_BOOM_BRIDGE         = 31
};

enum RF_PKT_IDX
{   // Indexes into RF Received pkt
    //  (fixed length)
    RF_IDX_LEN                      = 0,
    RF_IDX_SN_MSB                   = 1,
    RF_IDX_SN_MID                   = 2,
    RF_IDX_SN_LSB                   = 3,
    RF_IDX_DATA                     = 4,
    RF_IDX_CRC                      = 5,
    RF_IDX_PKT_SIZE                 = 6
};

enum RF_DOOR_WINDOW_STATUS
{   // Bit fields for Door/Window Status
    //  (fixed length)
    RF_DOOR_WINDOW_OPEN            = 0x01,
    RF_DOOR_WINDOW_EXT_CONTACT     = 0x02,
    RF_DOOR_WINDOW_TAMPERED        = 0x04,
    RF_DOOR_WINDOW_LOW_BATTERY     = 0x20,
    RF_DOOR_WINDOW_LEARN           = 0x40
};

enum RF_DEADBOLT_SENSOR_STATUS
{   // Bit fields for Deadbolt Status
    //  (fixed length)
    RF_DEADBOLT_SENSOR_OPEN        = 0x01,
    RF_DEADBOLT_SENSOR_UNLOCKED    = 0x02,
    RF_DEADBOLT_SENSOR_TAMPERED    = 0x04,
    RF_DEADBOLT_SENSOR_LOW_BATTERY = 0x20,
    RF_DEADBOLT_SENSOR_LEARN       = 0x40
};

enum RF_SIREN_STATUS
{   // Bit fields for Siren Status
    //  (fixed length)
    RF_SIREN_ON                    = 0x01,
    RF_SIREN_LOW_BATTERY           = 0x20,
    RF_SIREN_LEARN                 = 0x40
};

enum RF_STATUS_MODULE_STATUS
{   // Bit fields for StatusModule Status
    //  (fixed length)
    RF_STATUS_MODULE_LEARN          = 0x40,
    RF_STATUS_MODULE_REQ            = 0xa5,    // Sent by StatusModule to Boomerang, request info
    RF_STATUS_MODULE_ACK            = 0xaa,
    RF_STATUS_MODULE_INSTALL_MODE       = 0xa0,
    RF_STATUS_MODULE_INSTALL_MODE_END   = 0xa1,
    RF_STATUS_MODULE_TEST_MODE          = 0xa2,
    RF_STATUS_MODULE_TEST_MODE_END      = 0xa3,
    RF_STATUS_MODULE_STATUS_MODE        = 0xa4,
    RF_STATUS_MODULE_STATUS_MODE_END    = 0xa5,
    RF_STATUS_MODULE_LEARN_SUCCESS      = 0xb0,
    RF_STATUS_MODULE_LEARN_FAIL         = 0xb1,
    RF_STATUS_MODULE_TEST_1_BAR         = 0xb2,
    RF_STATUS_MODULE_TEST_2_BAR         = 0xb3,
    RF_STATUS_MODULE_TEST_3_BAR         = 0xb4,
    RF_STATUS_MODULE_TEST_4_BAR         = 0xb5,     // don't go past 0xb7, bit 3 is used for Low Battery
    RF_STATUS_MODULE_TEST_LOW_BATT_MASK = 0x08,
    RF_STATUS_MODULE_ALL_CLOSED_ALL_LOCKED      = 0xc0,
    RF_STATUS_MODULE_ALL_CLOSED_SOME_UNLOCKED   = 0xc1,
    RF_STATUS_MODULE_SOME_OPENED_ALL_LOCKED     = 0xc2,
    RF_STATUS_MODULE_SOME_OPENED_SOME_UNLOCKED  = 0xc3,
    RF_STATUS_MODULE_NO_CLOSED_ALL_LOCKED       = 0xc4,
    RF_STATUS_MODULE_NO_CLOSED_SOME_UNLOCKED    = 0xc5,
    RF_STATUS_MODULE_ALL_CLOSED_NO_LOCKED       = 0xc6,
    RF_STATUS_MODULE_SOME_OPENED_NO_LOCKED      = 0xc7,
    RF_STATUS_MODULE_NO_CLOSED_NO_LOCKED        = 0xc8,
    RF_STATUS_MODULE_GENERAL_FAULT_MASK         = 0x20  //bit 5 is used for 0xCx Status command
};


enum CHIME_SETTINGS
{   // Chime settings
    CHIME_NONE,
    CHIME_ON_OPEN,
    CHIME_ON_UNLOCK,
    CHIME_ON_OPEN_OR_UNLOCKED
 };

enum CHIME_CAUSE
{
    CHIME_FROM_OPEN,
    CHIME_FROM_UNLOCK,
    CHIME_FROM_OPEN_AND_LOW_BATT,
    CHIME_FROM_UNLOCK_AND_LOW_BATT,
    CHIME_FROM_LOW_BATT
};


#define MAX_DEVICES 128

// Timer1 defines used by W_Timer_Software1MilliSecDelay()
#define T1_PRESCALE                 256
#define T1_TOGGLES_PER_SEC          1000
#define T1_1MS_TICK                 (GetSystemClock()/T1_PRESCALE/T1_TOGGLES_PER_SEC)
#define T1_1SEC_TICK                (GetSystemClock()/1/1)

#define LEARN_TIMEOUT_IN_MINUTES    2

enum ZWAVE_REQUESTS
{
    ZWAVE_HUB_LEARN,            // Hub to ZWave
    ZWAVE_SENSOR_LEARN,         // Hub to ZWave
    ZWAVE_SENSOR_SENDDATA,      // Hub to ZWave
    ZWAVE_SENSOR_RECEIVE        // ZWave to Hub, For future 2-way
};

enum UI_STATES
{
    LED_BLUE,
    LED_RED,
    LED_GREEN,
    BUTTON_LEARN,
    BUTTON_TEST,
    BEEPER,
    CHIME,
    DELAY,
    SIREN,
    //LOS,
    //ZWAVE_ERR,

    MAX_UI
};

enum GENERAL_FAULTS
{
    GENERAL_FAULT_LOS =         0,
    GENERAL_FAULT_ZWAVE_ERR =   1,
    GENERAL_FAULT_DEVICE_LOW_BATT = 2,
    GENERAL_FAULT_BRIDGE_LOW_BATT = 3,
    TOTAL_GENERAL_FAULTS
};

struct ZWAVE_SENSOR_LEARN_REQUEST
{
    unsigned long SN;   //3 byte SN/devType
};

struct ZWAVE_HUB_LEARN_RESPONSE
{
    unsigned char result;
    unsigned char hubNodeID;
};

struct ZWAVE_SENSOR_LEARN_RESPONSE
{
    unsigned char result;
    unsigned char sensorNodeID;
};

struct ZWAVE_SENSOR_SENDDATA_RESPONSE
{
    unsigned char result;
};

struct NODE_ID_TO_SN
{
    unsigned char id;
    unsigned long SN;
    enum DEVICE_TYPE device_type;
    unsigned char curr_status;
    clock_t         receiveTime;
    unsigned char supervisorMissed;
    //unsigned char associatedNodeID;
    unsigned char last_insynctive_command_sent;
    unsigned char last_insynctive_param_sent;
    bool super_fault_status;
    unsigned char batt_level;
};                              //nodeIDtoSN[MAX_DEVICES];

#define MAX_STATUS_MODULES  8
struct STATUS_MODULE
{
    unsigned char id;
    unsigned long   SN;
    unsigned char   curr_status;
    DWORD           modeTime; //timeout for Status, Learn & Test modes
};                            //statusModule[MAX_STATUS_MODULES];

struct UI_STATE
{
    BOOL            active;
    unsigned char   on;
    unsigned int    counter;
    BOOL            appendLowBatt_toChime;  //applies only to Chime
    BOOL            lowBatt_only;           //applies only to Chime
    //BOOL            err_interrupted;        //applies only to Error UI states
};  //UIstate[MAX_UI];

struct GENERAL_FAULT
{
    BOOL            active;
    unsigned int    counter;
};

// Radio definitions
#ifdef HW_JAN_2013
#define NSS     BIT_8
#define DIO0    BIT_10
#else
#define NSS     BIT_10
#define DIO0    BIT_8
#endif
#define SCK     BIT_2
#define MOSI    BIT_3
#define MISO    BIT_4
#define RESET   BIT_7

#ifdef HW_JAN_2013
#define PACKET_RECEIVED mPORTDReadBits(DIO0)
#else
#define PACKET_RECEIVED mPORTBReadBits(DIO0)
#endif

//Packet retransmit variables
#define MAX_TX_QUEUE    5

struct TX_PKT_BUFFER
{
    unsigned char rfData[RF_IDX_PKT_SIZE];
    unsigned char retries;
    //BOOL resend;
    unsigned long SN;
    unsigned long tx_ack_timer;
};                              //txPktBuffer[MAX_TX_QUEUE];

enum ATTRIBUTE
{

    BATT_LEVEL = 1,   // 1 - Battery Level (0-100%)
    IS_OPEN,          // 2 - Is Open (true/false)
    IS_LOCKED,        // 3 - Is Locked (true/false)
    POSITION,         // 4 - Position (0-100%)
    ANGLE,            // 5 - Angle (-90 to 90 degrees)
    LOWER_SOFT_LIMIT, // 6 - Lower Soft Limit (0-100%)
    UPPER_SOFT_LIMIT, // 7 - Upper Soft Limit (0-100%)
    SAVED_POSITION,   // 8 - Saved Position (0-100%)
    IS_TAMPERED,      // 9 - Is Tampered (true/false)
    TARGET_POSITION,  // 10
    TARGET_ANGLE,     // 11
    INSERTION,        // 12 - To track insertion
    REMOVED,          // 13 - To track removal
    IS_SUPER_FAULT,   // 14 - Is in Supervisory Fault (true/false)
    SHADE_ACK         // 15 - Shade sends acknowledgement (true/false)

};

enum DbCommand
{
    SET = 0x1,
    GET, //2
    ENTER_LEARN, //3
    EXIT_LEARN, //4
    FORGET_DEVICE, //5
    ENTER_STATUS_MODULE_LEARN, //6
    ENTER_ALL_BUT_SM, //7
    ZWAVE_LEARN_SUCCESS, //8
    ZWAVE_LEARN_FAILED, //9
    CLEAR_MEMORY, //10
    ENTER_DIAG_MODE, //11
    EXIT_DIAG_MODE, //12
    ZWAVE_UNLEARN_SUCCESS, //13
    ZWAVE_ERROR, //14
    SUPERVISORY_FAULT, //15
    LEARN_ACK, //16
    LEARN_NAK, //17
    UNLEARN_ACK, //18
    UNLEARN_NAK, //19
};
/*
enum PositionValues
{
    SHADE_100_PERCENT   = 0x64,
    SHADE_LOWER_SOFT_LIMIT = 0x65,
    SHADE_UPPER_SOFT_LIMIT = 0x66,
    SHADE_SAVED_POSITION = 0x67,
    SHADE_HALT_MOVEMENT = 0x68
};*/

enum PACKET_FIELD
{
    SOH = 0x01,
    STX = 0x02,
    ETX = 0x03,
    ESC = 0x1b
};

enum UART_COMMANDS
{
    IMX_SPI_RD_REG = 0x20,
    IMX_SPI_WR_REG = 0x21,
    IMX_INSYNC_TX = 0x22,
    IMX_INSYNC_SHADE_TX = 0x23,
    IMX_VERSION_REQ = 0x24,
    IMX_RADIO_CHIP_REV_ID_REQ = 0x25,
    IMX_SET_GPIO_STATE = 0x26,
    PIC_SPI_RD_REG_RESP = 0x40,
    PIC_RF_RCVD_PKT = 0x41,
    PIC_END_OF_TX = 0x42,
    PIC_VERSION_RESP = 0x43
};


#define PIC_RESP_IND  1
#define INSYN_LEARN_COMPLETE 3

/** M A C R O S **************************************************************/
#define MAX_RCVD_BYTES 16
#define TickConvertToMilliseconds(t) (t * 1000)

// gets the number of seconds used
#define TickGet()                   ((double)clock()/sysconf(_SC_CLK_TCK))

// Represents one second in Ticks
#define TICK_SECOND             (sysconf(_SC_CLK_TCK))
// Represents one minute in Ticks
#define TICK_MINUTE             ((QWORD)TICK_SECOND * 60ull)
// Represents one hour in Ticks
#define TICK_HOUR               ((QWORD)TICK_SECOND * 3600ull)

//Shade retransmit interval
#define SHADE_RETRANSMIT_INTERVAL  125  //125msec
#define LEARN_TIMEOUT       (2) // 2min

#define delay_ms(s)  //usleep(s*1000)
#define MIN_TO_SECS(x)   (x*60)
#define MILLI_TO_NANOSEC(x)  (x*1000000)

/** P R O T O T Y P E S ******************************************************/
// main
void setFactoryDefaults(void);
void removeDeviceFromList(pthread_mutex_t *, unsigned long deviceSN, unsigned char id);
void removeStatusModuleFromList(unsigned long statusSN);
bool handle_shade_to_zwave(pthread_mutex_t *);

void setStatusByte(void);
void notifyStatusModule(void);
unsigned char get_nodeStatus(unsigned long pkt_sn);
void Start_GeneralFault(enum GENERAL_FAULTS type_generalFault);
void Stop_GeneralFault(enum GENERAL_FAULTS fault_type);
void Handle_GeneralFault(void);
void Retransmit();
void UartInit(int port);

/******************** getVirtualDeviceState ******************************************
 * Gets the device infromation from the interestingly named nodeIDtoSN array of also
 * interestingly named NODE_ID_TO_SN structs.
 *
 * @param   NODE_ID_TO_SN*  a pointer to a NODE_ID_TO_SN data structure to store a copy of the found info
 * @param   uint8_t         nodeID    the node ID to look up
 * @return  bool            true of nodeID was found
 *******************************************************************************/
bool getVirtualDeviceState(struct NODE_ID_TO_SN *record, uint8_t nodeID );
void AddToNodeIDTable(struct NODE_ID_TO_SN *n);
void AddStatusModule(unsigned char id, unsigned long   SN);

extern BYTE boomSN_msb;
extern BYTE boomSN_mid;
extern BYTE boomSN_lsb;
extern unsigned long boomSN;

void save_command_and_param( unsigned id, uint8_t cmd, uint8_t param );

// Ecolink Transceiver
void Ecolink_Tx(BYTE pkt_size);
void Ecolink_Tx_wait_ACK(BYTE pkt_size, unsigned long target_sn);
void prepPkt(BYTE pkt_size);
void Ecolink_Tx_Shade( void );
unsigned char  Ecolink_Rx(unsigned char* buf);

// Radio
void radio_init(int port);
void radio_receive_packet(void);
void radio_transmit_packet(unsigned char rf_byte[], int num);
void radio_data_in(uint8_t address);
int radio_read(unsigned char* data_ptr, int maxBytes);
void radio_pic_version_read();
void radio_powerlevel_set();
void* RadioThread(void*);
void HandleEndOfTx();


#endif
