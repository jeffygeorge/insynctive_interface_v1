#define _GNU_SOURCE

/*
  ;
  ;   Timer Usage:
  ;
  ;   CoreTimer   100ms   Core System timer, used for UI states (LEDs and Beeper on/off), diagnostic
  ;                           timeouts and Supervisor
  ;   Timer1      -----   used by Ethernet Stack
  ;   Timer2      3.2us   used for Beeper PWM
  ;   Timer3      3.2us   used for LED PWM
  ;   Timer4      100ns   used for us & ms delays in RF Rx
  ;   Timer5              unused
*/

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <syslog.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "DbInterface.h"
#include "EventQ.h"
#include "shade_transmit_queue.h"
#include "Timer.h"

#include "ESW1032P-01.h"

#define RSSI_REG       0x11
#define MAJOR_VERSION  1
#define MINOR_VERSION  1
#define BUILD_VERSION  6

unsigned char learnedDeviceCount;
unsigned char myNodeID;
unsigned char myNodeID_idx;
unsigned long learnPktSN;
unsigned char learnPktStatus;
unsigned long pktSN;
unsigned long boomSN;

BYTE boomSN_msb;
BYTE boomSN_mid;
BYTE boomSN_lsb;
BYTE rssi_val;

enum DEVICE_TYPE currently_learning_device_type;
enum DEVICE_TYPE device_type;

unsigned long currently_learning_sn;

// Timer for check on shade potison
DWORD shadeReportTime = 0x00000000;

// Used for Supervisor & ZWave mgmnt
clock_t   boomTicks = 0x00000000;

bool    LOS_state = false;
DWORD   deltaTime;
bool entering_new_state = false;
static enum STATE_MACHINE_STATES last_state ;
static enum STATE_MACHINE_STATES state;
static enum STATE_MACHINE_STATES learnState;
bool bUnlearn, bUnlearnStatusModule;
static bool Start = true;
static bool LearnAllButStatusModules = false;
static bool ZWave_err_state = false;
static bool delay_shade_transmit = false;
bool checkShade = false;

// RF receive variables
unsigned short crcWord;
unsigned char crcByte;

unsigned char rcvd_bytes[MAX_RCVD_BYTES];
unsigned char savedRcvd_bytes[MAX_RCVD_BYTES]; // Saved bytes to learn device
extern unsigned char rf_bytes[66];

// Open/lock variables
BYTE unlockedCount;
BYTE openedCount;
BYTE lockDevices;
BYTE openDevices;

// Device low battery variable
BYTE lowBattCount;

// Status Module variables
BYTE activeStatusModules;

// General fault management
struct GENERAL_FAULT generalFault[TOTAL_GENERAL_FAULTS];
bool err_interrupted;
DWORD general_fault_reload_time;
bool forceGenFaultStart = false;
BYTE faultRepeats;
BYTE currentFault;

struct NODE_ID_TO_SN nodeIDtoSN[MAX_DEVICES];
struct STATUS_MODULE statusModule[MAX_STATUS_MODULES];

BYTE txPktQueueCount;

// Diagnostic mode variables
DWORD diagTimeStart, diagTimeDelta;
bool startDiag;
bool force_diagExit = false;

unsigned char rf_pktCount;

// BoomBridge battery checking
unsigned int offset;

bool handle_shade_to_zwave(pthread_mutex_t *);
bool virtualReceive(unsigned long pkt_sn);
void send_queued_packets(void);
static void HandleDBCommands(pthread_mutex_t *, const struct DbEvent *evt);
static void SendInstallStatus(unsigned char status);
static void PrintVersion();
static bool isStatusModule(unsigned char id, int *ind);
static void StartRssiTimer();
void HandleShadeAttribute(const struct DbEvent *evt, unsigned long SN);
bool IsDeviceKnown(unsigned long serial_number);

// Old bytes from devices. Currently limited to 10 sets of bytes

#define MAXOLDBYTES (10)

struct sent_bytes{
    unsigned char bytes[MAXOLDBYTES][MAX_RCVD_BYTES];
    int front;
    int count;
} oldBytes;


DWORD first_pkt_time;
unsigned long test_sn;
BYTE test_data;
unsigned int rssi_avg;
BYTE rssi[8];

bool isOpened = false;
bool isClosed = false;
bool isLocked = false;
bool isUnlocked = false;

unsigned int channel0;    // conversion result as read from result buffer

static void CreateThreads(pthread_mutex_t *mutex);
static void handleIncomingRFPacket(pthread_mutex_t *, const char *deviceSn);
void CheckStatusModuleTimeout(void);
void ResetStatusModuleStatus(int i);

static timer_t s_RxTimer;
static timer_t s_StatusModuleTimer = 0;

#define ShadeTimeLimit (45 * TICK_SECOND)
#define MAX_DEVICE_TYPES  3
#define SENSOR_LOW_BATT_VALUE 0
#define SENSOR_NO_LOW_BATT_VALUE 100
#define STATUS_MODULE_TIMEOUT_SEC (1*60)

bool haveSpaceForAnotherDevice() {
    return (learnedDeviceCount < (MAX_DEVICES - 1));
}

bool haveSpaceForAnotherStatusModule() {
    return (activeStatusModules < (MAX_STATUS_MODULES - 1));
}

void clear_receive_buffer(void)
{
    memset(rcvd_bytes, 0, MAX_RCVD_BYTES);
}

// Check to see if the device already know to the bridge
bool IsDeviceKnown(unsigned long serial_number)
{
    if (learnedDeviceCount >= MAX_DEVICES)
        learnedDeviceCount = MAX_DEVICES - 1;

    for (int i = 0; i < learnedDeviceCount; ++i)
        if (nodeIDtoSN[i].SN == serial_number)
            return true;

    return false;
}

//Sets all ti= 0;
void resetOldBytes(void)
{
    oldBytes.front = 0;
    oldBytes.count = 0;

    memset(oldBytes.bytes, 0, MAXOLDBYTES * MAX_RCVD_BYTES);
}

void addBytes(void)
{
    if (oldBytes.count == MAXOLDBYTES) {
        for (int j = 0; j < (int)(rcvd_bytes[0] + 1); j++)
            oldBytes.bytes[oldBytes.front][j] = rcvd_bytes[j];
        oldBytes.front = (oldBytes.front + 1) % MAXOLDBYTES;
    } else {
        for (int j = 0; j < (int)(rcvd_bytes[0] + 1); j++)
            oldBytes.bytes[oldBytes.front + oldBytes.count][j] = rcvd_bytes[j];
        oldBytes.count++;
    }
}

// Look to see if the new bytes match any byte sent in the past 10
// times. Includes the length of the bytes to be searched
bool containBytes(void)
{
    int matchCount = 0;

    for (int i = 0; i < oldBytes.count; i++) {
        matchCount = 0;

        for (int j = 0; j < (int)(rcvd_bytes[0] + 1); j++)
            if (rcvd_bytes[j] == oldBytes.bytes[i][j])
                matchCount++;

        if (matchCount == (int)(rcvd_bytes[0] + 1))
            return 1;
    }

    // If the bytes are not contained add them.
    addBytes();

    return 0;
}

char nibbleToHexCharacter(uint8_t nibble)
{
    switch (nibble && 0x0F) {
    case 0x0:
    case 0x1:
    case 0x2:
    case 0x3:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
    case 0x8:
    case 0x9:
        return '0' + nibble;
    case 0xA:
    case 0xB:
    case 0xC:
    case 0xD:
    case 0xE:
    case 0xF:
        return 'A' + nibble - 10;
    default:
        return ' ';
    }
}

uint8_t hexCharacterToNibble(char nibble)
{
    switch (toupper(nibble)) {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
        return nibble - '0';
    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
    case 'F':
        return nibble - 'A' + 10;
    default:
        return 0;
    }
}

char *idToHexString(unsigned long id)
{
    char *retval = NULL;
    int status = asprintf(&retval, "0x%06lX", id);

    if (status == -1) {
        syslog(LOG_ERR, "idToHexString(): Error from asprintf()");
        return 0;
    }

    return retval;
}

void hexStringToBytes(const char *src, unsigned char *dest)
{
    int len = ((hexCharacterToNibble(src[0]) << 4) | hexCharacterToNibble(src[1])) + 1;

    for (int i = 0; i < len; i++)
        dest[i] = (hexCharacterToNibble(src[i * 2]) << 4) | hexCharacterToNibble(src[i * 2 + 1]);
}

void handleIncomingRFPacket(pthread_mutex_t *mutex, const char *playbackData)
{
    int i;
    bool received_bad_packet = false;

    isOpened = false;
    isClosed = false;
    isLocked = false;
    isUnlocked = false;

    // Treat any playback data passed in to the function as having
    // been received over the RF interface
    if (playbackData)
        hexStringToBytes(playbackData, rcvd_bytes);

    // Pull out the serial number/device code from RF pkt
    pktSN =
        rcvd_bytes[RF_IDX_SN_MSB] << 16 |
        rcvd_bytes[RF_IDX_SN_MID] << 8  |
        rcvd_bytes[RF_IDX_SN_LSB] << 0;

    device_type = (enum DEVICE_TYPE)(rcvd_bytes[RF_IDX_SN_MSB] >> 3);

    // Check to see if the device is already paired
    if (!containBytes() || playbackData || Start) {
        // We're going to further filter what we are receiving to
        // minimize seeing noise as data

        switch(device_type) {

        case DEVICE_TYPE_DOOR_WINDOW_SENSOR:
        case DEVICE_TYPE_TILT_SENSOR:
        case DEVICE_TYPE_DEADBOLT_SENSOR:
            if ((rcvd_bytes[RF_IDX_LEN] != 5) &&
                (rcvd_bytes[RF_IDX_LEN] != 7)) {

                syslog(LOG_ERR, "Bad DW/Tilt/DB Packet");
                received_bad_packet = true;
            }
            break;

        case DEVICE_TYPE_WINDOW_SHADE:
        case DEVICE_TYPE_BOOM_BRIDGE:
            if ((rcvd_bytes[RF_IDX_LEN] != 5) &&
                (rcvd_bytes[RF_IDX_LEN] != 6)) {

                syslog(LOG_ERR, "Bad shade packet");
                received_bad_packet = true;
            }

            break;

        case DEVICE_TYPE_STATUS_MODULE:
            if (rcvd_bytes[RF_IDX_LEN] != 5) {
                syslog(LOG_INFO, "Bad status module packet");
                received_bad_packet = true;
            }

            break;

        case DEVICE_TYPE_INSYNCTIVE_BLIND_REMOTE:
            break;

        default:
            syslog(LOG_ERR, "Received unknown RF packet");
            received_bad_packet = true;
        }

        if (received_bad_packet) {
            clear_receive_buffer();
            return;
        }

        // Is there a Status Module in Test Mode?
        if (activeStatusModules > 0) {
            for (i = 0; i < activeStatusModules; i++) {
                if ((statusModule[i].curr_status == RF_STATUS_MODULE_TEST_MODE) &&
                    (statusModule[i].SN != pktSN)) {

                    // If this is the first pkt, start 1.6 second
                    // timer to collect all pkts per logical
                    // transmission

                    if (rf_pktCount == 0) {

                        TimerStop(s_RxTimer);
                        TimerStart(s_RxTimer, 1, 600, 0, 0);
                        radio_data_in(RSSI_REG);
                        test_sn = pktSN;
                        test_data = rcvd_bytes[RF_IDX_DATA];
                        rssi[rf_pktCount] = rssi_val;
                        rf_pktCount++;

                    } else {

                        // Make sure it's same Tx SN / payload as our
                        // starting pkt

                        if ((pktSN == test_sn) &&
                            (test_data == rcvd_bytes[RF_IDX_DATA])) {

                            rssi[rf_pktCount] = rssi_val;
                            rf_pktCount++;
                        }
                    }

                    // Has 1.6 second timer expired?

                    if (s_RxTimer && !TimerIsActive(s_RxTimer)) {

                        // Calculate RSSI

                        for (i = 0, rssi_avg = 0; i < rf_pktCount; i++)
                            rssi_avg += rssi[i];
                        rssi_avg /= rf_pktCount;

                        if (rssi_avg < 140)
                            rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_TEST_4_BAR;
                        else if (rssi_avg <= 160)
                            rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_TEST_3_BAR;
                        else if (rssi_avg <= 180)
                            rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_TEST_2_BAR;
                        else
                            rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_TEST_1_BAR;

                        // Check for device low battery

                        if (test_data & RF_DOOR_WINDOW_LOW_BATTERY)
                            rf_bytes[RF_IDX_DATA] |= RF_STATUS_MODULE_TEST_LOW_BATT_MASK;

                        rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
                        rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
                        rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;

                        Ecolink_Tx_wait_ACK(6, statusModule[i].SN);
                        syslog("insynctive-interface-jeffy", "this is in main function");
                        StartRssiTimer();

                        rf_pktCount = 0;
                    }
                }
            }
        }

        if (state != STATE_MACHINE_DIAG_DEVICE_TO_HUB) {

            // Device type dictates status and payload content

            switch(device_type) {
            case DEVICE_TYPE_INSYNCTIVE_BLIND_REMOTE:
                /* Remote for blinds and shade. Sets the
                   acknowledgement of the shade to false until the
                   shade or blind acknowledge the move.  Loops through
                   known devices */
                break;

            case DEVICE_TYPE_DOOR_WINDOW_SENSOR:
            case DEVICE_TYPE_TILT_SENSOR:
            {
                // Look for Learn bit in payload
                if (((rcvd_bytes[RF_IDX_DATA] == 0x40) ||
                     (rcvd_bytes[RF_IDX_DATA] == 0x41)) &&
                    (state != STATE_MACHINE_LEARN_STATUS_MODULE)) {

                    /* This is a learn/unlearn request */

                    /* Is sensor already learned? */
                    for (i = 0, bUnlearn = false; i < learnedDeviceCount; i++)
                        if (nodeIDtoSN[i].SN == pktSN) {
                            bUnlearn = true;
                            break;
                        }

                    // If already in the process of trying to learn
                    // this SN in, save the status and just exit

                    // TBD: this will be removed when the Ecolink
                    // protocol sends a learn pkt w/o any status

                    //BG: The only reason learn is sucessfull is
                    //because of this line...
                    if (learnPktSN == pktSN) {
                        learnPktStatus = rcvd_bytes[RF_IDX_DATA] & ~(RF_DOOR_WINDOW_LEARN);
                        break;
                    }

                    // learn device through button press and learn hub on power up
                    if ((state == STATE_MACHINE_LEARN_DEVICE) ||
                        LearnAllButStatusModules) {
                        if (!bUnlearn) {
                            learnPktSN = pktSN;
                            state = STATE_MACHINE_LEARN_SUCCESS;

                        } else {
                            char *data;

                            asprintf(&data, "0x%06lX", pktSN);
                            DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, data);
                            learnPktSN = pktSN;
                            bUnlearn = true;
                            state = STATE_MACHINE_UNLEARN_SUCCESS;
                        }
                    }

                } else {

                    /* Handle something other than learn/unlearn */

                    //Look up SN, make sure it's a learned nodeID
                    if (virtualReceive(pktSN)) {

                        // Update Supervisor
                        if (boomTicks-nodeIDtoSN[myNodeID_idx].receiveTime)
                            nodeIDtoSN[myNodeID_idx].receiveTime = boomTicks;

                        ++nodeIDtoSN[myNodeID_idx].supervisorMissed;

                        {
                            // Low battery condition should always be
                            // updated before anything else so the
                            // appropriate chimes on a unlock will
                            // sound

                            unsigned char battStatus =
                                (rcvd_bytes[RF_IDX_DATA] & RF_DOOR_WINDOW_LOW_BATTERY) ?
                                SENSOR_LOW_BATT_VALUE : SENSOR_NO_LOW_BATT_VALUE;

                            // Keep track of device Low battery status
                            if (!(nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_LOW_BATTERY) &&
                                (battStatus == SENSOR_LOW_BATT_VALUE)) {

                                /* Battery just went low... */

                                /* Increment with overflow protection */
                                if (0 == (++lowBattCount)) --lowBattCount;

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             BATT_LEVEL, SENSOR_LOW_BATT_VALUE,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime,
                                             false);

                                nodeIDtoSN[myNodeID_idx].batt_level = SENSOR_LOW_BATT_VALUE;
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;

                            } else if ((nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_LOW_BATTERY) &&
                                       (battStatus == SENSOR_NO_LOW_BATT_VALUE)) {

                                /* Battery was low but isn't now */

                                if (lowBattCount > 0) --lowBattCount;//decrement with underflow protection

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             BATT_LEVEL, SENSOR_NO_LOW_BATT_VALUE,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime,
                                             false);

                                nodeIDtoSN[myNodeID_idx].batt_level = SENSOR_NO_LOW_BATT_VALUE;
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            }

                            // Check for Tamper
                            if ((rcvd_bytes[RF_IDX_DATA] & RF_DOOR_WINDOW_TAMPERED) &&
                                !(nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_TAMPERED)) {

                                isOpened = true;

                                if ((++openedCount) == 0) --openedCount;//increment with rollover protection

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id, nodeIDtoSN[myNodeID_idx].device_type,
                                             IS_TAMPERED, true, &nodeIDtoSN[myNodeID_idx].receiveTime, false);

                                char *data;
                                asprintf(&data, "%lu", nodeIDtoSN[myNodeID_idx].SN);
                                DbAddToBLCmdTable(NULL, mutex, BL_INDICATE_TAMPERED, data);

                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            } else if (!(rcvd_bytes[RF_IDX_DATA] & RF_DOOR_WINDOW_TAMPERED) &&
                                       (nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_TAMPERED)) {

                                isClosed = true;

                                if (openedCount > 0) --openedCount;//Decrement with rollver protection

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             IS_TAMPERED, false,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime, false);

                                char *data;
                                asprintf(&data, "%lu", nodeIDtoSN[myNodeID_idx].SN);
                                DbAddToBLCmdTable(NULL, mutex, BL_INDICATE_UNTAMPERED, data);
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            }

                            if ((rcvd_bytes[RF_IDX_DATA] & RF_DOOR_WINDOW_OPEN) &&
                                !(nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_OPEN)) {

                                isOpened = true;

                                if ((++openedCount) == 0) --openedCount;//increment with rollover protection

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             IS_OPEN, true,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime, false);
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            } else if (!(rcvd_bytes[RF_IDX_DATA] & RF_DOOR_WINDOW_OPEN) &&
                                       (nodeIDtoSN[myNodeID_idx].curr_status & RF_DOOR_WINDOW_OPEN)) {

                                isClosed = true;

                                if (openedCount > 0) --openedCount;//Decrement with rollver protection

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             IS_OPEN, false,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime, false);
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            }


                            if (nodeIDtoSN[myNodeID_idx].supervisorMissed > 0) {

                                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                             nodeIDtoSN[myNodeID_idx].device_type,
                                             BATT_LEVEL, battStatus,
                                             &nodeIDtoSN[myNodeID_idx].receiveTime,
                                             false);

                                nodeIDtoSN[myNodeID_idx].batt_level = battStatus;
                                nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                            }
                        }

                        // Check for Chime
                        if (isOpened) {
                            // If any active Status modules, notify them of status change
                            notifyStatusModule();
                            isOpened = false;
                        } else if (isClosed) {
                            // If any active Status modules, notify them of status change
                            notifyStatusModule();

                            //law TBD
                            // Learn is screwing us up
                            if (openedCount == 0xff)//BG: This is NOT good over flow protection. I am not sure why this is here.
                                openedCount = 0;
                            //law TBD
                            isClosed = false;
                        }

                        if (lowBattCount != 0) {
                            Start_GeneralFault(GENERAL_FAULT_DEVICE_LOW_BATT);
                        } else {
                            Stop_GeneralFault(GENERAL_FAULT_DEVICE_LOW_BATT);
                        }

                        nodeIDtoSN[myNodeID_idx].curr_status = rcvd_bytes[RF_IDX_DATA];
                    }
                }//Learn bit not set

                break;
            } //DOOR_WINDOW

            case DEVICE_TYPE_DEADBOLT_SENSOR:
            {
                // Look for Learn bit in payload
                if (((rcvd_bytes[ RF_IDX_DATA ] == 0x40) ||
                     (rcvd_bytes[ RF_IDX_DATA ] == 0x42)) &&
                    (state != STATE_MACHINE_LEARN_STATUS_MODULE))
                {
                    //See if already learned in
                    for (i = 0, bUnlearn = false; i < learnedDeviceCount; i++)
                        if (nodeIDtoSN[i].SN == pktSN) {
                            bUnlearn = true;
                            break;
                        }

                    // If already in the process of trying to learn this SN in, save the status and just exit
                    // TBD, eventually this will be removed since the final Ecolink protocol will
                    //      send a learn pkt w/o any status

                    //BG: The only reason learn is successful is because of this line...
                    if (learnPktSN == pktSN) {
                        learnPktStatus = rcvd_bytes[RF_IDX_DATA] & ~(RF_DEADBOLT_SENSOR_LEARN);
                        break;
                    }

                    if ((state == STATE_MACHINE_LEARN_DEVICE) ||
                        LearnAllButStatusModules) {
                        // Standalone Learn requires that we are
                        // already in state == STATE_MACHINE_LEARN_DEVICE

                        if (!bUnlearn) {
                            learnPktSN = pktSN;
                            state = STATE_MACHINE_LEARN_SUCCESS;
                        } else {
                            char *data;

                            asprintf(&data, "0x%06lX", pktSN);
                            DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, data);
                            learnPktSN = pktSN;
                            bUnlearn = true;
                            state = STATE_MACHINE_UNLEARN_SUCCESS;
                        }
                    }
                } else {
                    //Look up SN, make sure it's a learned nodeID
                    if (virtualReceive(pktSN))
                    {   // learned nodeID, send BasicSet for Open/Close
                        // Update Supervisor
                        nodeIDtoSN[myNodeID_idx].receiveTime = boomTicks;
                        ++nodeIDtoSN[myNodeID_idx].supervisorMissed;

                        // Low battery condition should always be updated before anything else so
                        // the appropriate chimes on a unlock will sound

                        unsigned char battStatus =
                            (rcvd_bytes[RF_IDX_DATA] & RF_DEADBOLT_SENSOR_LOW_BATTERY) ?
                            SENSOR_LOW_BATT_VALUE : SENSOR_NO_LOW_BATT_VALUE;

                        // Keep track of device Low battery status
                        if (!(nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_LOW_BATTERY) &&
                            (battStatus == SENSOR_LOW_BATT_VALUE)) {

                            /* Battery just went low... */

                            if (0 == (++lowBattCount)) --lowBattCount;//Increment with overflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         BATT_LEVEL, SENSOR_LOW_BATT_VALUE,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime,
                                         false);

                            nodeIDtoSN[myNodeID_idx].batt_level = SENSOR_LOW_BATT_VALUE;
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;

                        } else if ((nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_LOW_BATTERY) &&
                                   (battStatus == SENSOR_NO_LOW_BATT_VALUE)) {

                            /* Battery was low but isn't now */

                            if (lowBattCount > 0) --lowBattCount;//decrement with underflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         BATT_LEVEL, SENSOR_NO_LOW_BATT_VALUE,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime,
                                         false);

                            nodeIDtoSN[myNodeID_idx].batt_level = SENSOR_NO_LOW_BATT_VALUE;
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                        }

                        // Check for Tamper
                        //  Tamper on Deadbolt is just like Unlock
                        if ((rcvd_bytes[RF_IDX_DATA] & RF_DEADBOLT_SENSOR_TAMPERED) &&
                            !(nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_TAMPERED)) {

                            isUnlocked = true;

                            if ((++unlockedCount) == 0) --unlockedCount;//increment with overflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         IS_TAMPERED, true,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime, false);

                            char *data;
                            asprintf(&data, "%lu", nodeIDtoSN[myNodeID_idx].SN);
                            DbAddToBLCmdTable(NULL, mutex, BL_INDICATE_TAMPERED, data);
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;

                        } else if (!(rcvd_bytes[RF_IDX_DATA] & RF_DEADBOLT_SENSOR_TAMPERED) &&
                                   (nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_TAMPERED)) {

                            isLocked = true;

                            if (unlockedCount > 0) --unlockedCount;//decrement with underflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         IS_TAMPERED, false,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime, false);

                            char *data;
                            asprintf(&data, "%lu", nodeIDtoSN[myNodeID_idx].SN);
                            DbAddToBLCmdTable(NULL, mutex, BL_INDICATE_UNTAMPERED, data);
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                        }

                        // Deadbolt has no "Open" state, only "Unlock"
                        if ((rcvd_bytes[RF_IDX_DATA] & RF_DEADBOLT_SENSOR_UNLOCKED) &&
                            !(nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_UNLOCKED)) {

                            isUnlocked = true;

                            if ((++unlockedCount) == 0) --unlockedCount;//increment with overflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         IS_LOCKED, false,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime, false);
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;

                        } else if (!(rcvd_bytes[RF_IDX_DATA] & RF_DEADBOLT_SENSOR_UNLOCKED) &&
                                   (nodeIDtoSN[myNodeID_idx].curr_status & RF_DEADBOLT_SENSOR_UNLOCKED)/* &&
                                                                                                          state == STATE_MACHINE_IDLE*/)
                        {
                            isLocked = true;

                            if (unlockedCount > 0) --unlockedCount;//decrement with underflow protection

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         IS_LOCKED, true,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime, false);
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                        }

                        if (nodeIDtoSN[myNodeID_idx].supervisorMissed > 0) {

                            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                                         nodeIDtoSN[myNodeID_idx].device_type,
                                         BATT_LEVEL, battStatus,
                                         &nodeIDtoSN[myNodeID_idx].receiveTime,
                                         false);

                            nodeIDtoSN[myNodeID_idx].batt_level = battStatus;
                            nodeIDtoSN[myNodeID_idx].supervisorMissed = 0;
                        }

                        // Check for Chime
                        if (isUnlocked) {
                            // If any active Status modules, notify them of status change
                            notifyStatusModule();
                            isUnlocked = false;
                        } else if (isLocked) {
                            // If any active Status modules, notify them of status change
                            notifyStatusModule();
                            isLocked = false;
                        }

                        if (lowBattCount != 0) {
                            Start_GeneralFault(GENERAL_FAULT_DEVICE_LOW_BATT);
                        } else {
                            Stop_GeneralFault(GENERAL_FAULT_DEVICE_LOW_BATT);
                        }

                        nodeIDtoSN[myNodeID_idx].curr_status = rcvd_bytes[RF_IDX_DATA];
                    }
                }//Learn bit not set

                break;
            } //DEADBOLT

            case DEVICE_TYPE_STATUS_MODULE:

                /* Look for Learn bit in payload */
                if (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_LEARN) {

                    /* Learn */
                    if (state == STATE_MACHINE_LEARN_STATUS_MODULE ||
                        state == STATE_MACHINE_LEARN_DEVICE) {

                        /* See if already learned in */
                        for (i = 0, bUnlearn = false; i < activeStatusModules; i++) {
                            if (statusModule[i].SN == pktSN) {
                                char *data;

                                asprintf(&data, "0x%06lX", pktSN);
                                DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, data);

                                bUnlearn = true;
                                break;
                            }
                        }

                        if (learnPktSN == pktSN) {
                            //BG: The only reason learn is successful is because of this line...
                            break;
                        }

                        // Send back an ACK with Hub SN
                        delay_ms(5);//give time to Status Module to switch to receive mode

                        if (bUnlearn) {
                            rf_bytes[RF_IDX_SN_MSB] = 0xff;
                            rf_bytes[RF_IDX_SN_MID] = 0xff;
                            rf_bytes[RF_IDX_SN_LSB] = 0xff;
                        } else {
                            rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
                            rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
                            rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
                        }

                        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_ACK;
                        Ecolink_Tx(6);

                        // Not yet learned in, add Status Module
                        if (!bUnlearn) {

                            // Learn-Add to table
                            statusModule[activeStatusModules].SN = pktSN;
                            AddPairedDevice(NULL, mutex, pktSN, DEVICE_TYPE_STATUS_MODULE,
                                            &statusModule[activeStatusModules].id);

                            ++activeStatusModules;

                            //Inform BL about learn success
                            char *data;
                            asprintf(&data, "%06lx", pktSN);

                            DbAddToBLCmdTable(NULL, mutex, BL_LEARN_SUCCESS, data);

                        } else {
                            char *data;

                            asprintf(&data, "0x%06lX", pktSN);
                            DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, data);

                            // Unlearn-Remove from table
                            RemovePairedDevice(NULL, mutex, statusModule[i].id);
                            removeStatusModuleFromList(statusModule[i].SN);
                            learnPktSN = pktSN;

                            // Successful Learn/Unlearn
                            state = STATE_MACHINE_LEARN_SUCCESS;
                        }
                    }
                } else if (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_REQ) {
                    // Request
                    // Make sure requester is a learned in Status Module
                    for (i = 0; i < activeStatusModules; i++) {
                        if (statusModule[i].SN == pktSN) {
                            break;
                        }
                    }

                    // Send back response to Req with Hub SN
                    if (i != activeStatusModules) {
                        delay_ms(5); //give time to Status Module to switch to receive mode
                        rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
                        rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
                        rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
                        setStatusByte();
                        Ecolink_Tx(6);
                        if (s_StatusModuleTimer) {
                            TimerStop(s_StatusModuleTimer);
                            TimerStart(s_StatusModuleTimer, STATUS_MODULE_TIMEOUT_SEC, 0,0,0);
                        }
                    }
                } else if ((rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_STATUS_MODE) ||
                           (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_STATUS_MODE_END)) {

                    // Status Mode (start or end)
                    // Make sure requester is a learned in Status Module
                    for (i = 0; i < activeStatusModules; i++) {
                        if (statusModule[i].SN == pktSN) {
                            // Save state of Status Module
                            statusModule[i].curr_status = rcvd_bytes[RF_IDX_DATA];

                            // Send back an ACK with Hub SN
                            delay_ms(5); //give time to Status Module to switch to receive mode
                            rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
                            rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
                            rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
                            setStatusByte();
                            Ecolink_Tx(6);

                            if (s_StatusModuleTimer == 0)
                                TimerInit(&s_StatusModuleTimer, TIMER_EVENT);

                            TimerStart(s_StatusModuleTimer, STATUS_MODULE_TIMEOUT_SEC, 0,0,0);

                            // If start of mode, start timer
                            if (statusModule[i].curr_status == RF_STATUS_MODULE_STATUS_MODE) {
                                statusModule[i].modeTime = boomTicks;
                            }
                        }

                        rf_pktCount = 0;
                    }
                } else if ((rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_INSTALL_MODE) ||
                           (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_INSTALL_MODE_END) ||
                           (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_TEST_MODE) ||
                           (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_TEST_MODE_END)) {

                    // Install/Test Mode (start or end)
                    // Make sure requester is a learned in Status Module
                    for (i = 0; i < activeStatusModules; i++) {
                        if (statusModule[i].SN == pktSN) {
                            // Save state of Status Module
                            statusModule[i].curr_status = rcvd_bytes[RF_IDX_DATA];

                            // Send back an ACK with Hub SN
                            delay_ms(5); //give time to Status Module to switch to receive mode
                            rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
                            rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
                            rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
                            rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_ACK;
                            Ecolink_Tx(6);

                            // If start of mode, start timer
                            if ((statusModule[i].curr_status == RF_STATUS_MODULE_INSTALL_MODE) ||
                                (statusModule[i].curr_status == RF_STATUS_MODULE_TEST_MODE)) {
                                statusModule[i].modeTime = TickGet();
                            }
                        }

                        rf_pktCount = 0;
                    }
                }

                break;          /* DEVICE_TYPE_STATUS_MODULE */

            case DEVICE_TYPE_WINDOW_SHADE:

                if (virtualReceive(pktSN)) {
                    if (handle_shade_to_zwave(mutex)) {
                        if (rcvd_bytes[RF_IDX_DATA] == WINDOW_SHADE_CMD_S2B_ACK)
                            erase_remaining_sent_nonack_command_queued_for_shade(pktSN);

                        if ((rcvd_bytes[ RF_IDX_DATA ] == WINDOW_SHADE_CMD_S2B_LEARN_REQ) &&
                            (learnPktSN != pktSN)) {

                            if ((state == STATE_MACHINE_LEARN_DEVICE) ||
                                LearnAllButStatusModules) {
                                char *data;

                                asprintf(&data, "0x%06lX", pktSN);
                                DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, data);
                                learnPktSN = pktSN;
                                bUnlearn = true;
                                state = STATE_MACHINE_UNLEARN_SUCCESS;
                            }
                        }
                    }
                } else {
                    if (rcvd_bytes[ RF_IDX_DATA ] == WINDOW_SHADE_CMD_S2B_LEARN_REQ) {
                        if (((state == STATE_MACHINE_LEARN_DEVICE) || LearnAllButStatusModules) &&
                            (learnPktSN != pktSN)) {
                            learnPktSN = pktSN;
                            learnPktStatus = 0;
                            bUnlearn = false;
                            state = STATE_MACHINE_LEARN_SUCCESS;
                        }
                    }
                }

                break;          /* DEVICE_TYPE_WINDOW_SHADES */

            default:
                break;
            }
        } else {
            ++rf_pktCount;
        }

        if (nodeIDtoSN[myNodeID_idx].supervisorMissed == 0) {
            //Cheks to see if the device was in Supervisory fault to avoid repeating db
            // to write to the db
            if (nodeIDtoSN[myNodeID_idx].super_fault_status == true)
            {
                //Sets the device into not being in superviory fault
                nodeIDtoSN[myNodeID_idx].super_fault_status = false;

                //Writes to the db that the device is not in superviroy fault
                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                             nodeIDtoSN[myNodeID_idx].device_type,
                             IS_SUPER_FAULT, false,
                             &nodeIDtoSN[myNodeID_idx].receiveTime, false);
            }
            else
            {
            }
        }
    }
    else
    {
    }
}

void step_bridgeStateMachine(pthread_mutex_t *mutex) {

    unsigned int i;
    bool done = false;
#define IDLE_CHECK_TIMEOUT 10
    static timer_t s_IdleTimer = 0;

    if (s_IdleTimer == 0)
    {
        TimerInit(&s_IdleTimer, TIMER_EVENT);
    }

    switch (state) {
    case STATE_MACHINE_POR:
        // See if our Hub learned into ZWave Gateway
        state = STATE_MACHINE_IDLE; //STATE_MACHINE_LEARN_HUB;
        radio_pic_version_read();
        radio_data_in(RSSI_REG);
        radio_data_in(RSSI_REG);
        break;

    case STATE_MACHINE_LEARN_SUCCESS:

        /* If we don't have room for another device in our data
         * structures, don't try to learn a new one.  Note: need to
         * investigate processing of timers and other cleanup that may
         * be necessary; this is a simple fix to avoid a buffer
         * overflow. */

        if (!haveSpaceForAnotherDevice())
            break;

        // Entering state
        // If there is a Status Module, send result
        if (!LearnAllButStatusModules)
            SendInstallStatus(RF_STATUS_MODULE_LEARN_SUCCESS);

        currently_learning_device_type = (enum DEVICE_TYPE)(learnPktSN >> 19);

        // Learn-Add to table
        if (currently_learning_device_type != DEVICE_TYPE_STATUS_MODULE) {

            nodeIDtoSN[learnedDeviceCount].SN = learnPktSN;
            nodeIDtoSN[learnedDeviceCount].device_type = currently_learning_device_type;

            switch (currently_learning_device_type) {

            case DEVICE_TYPE_TILT_SENSOR:
            case DEVICE_TYPE_DOOR_WINDOW_SENSOR:
                nodeIDtoSN[learnedDeviceCount].curr_status = RF_DOOR_WINDOW_OPEN | RF_DOOR_WINDOW_LOW_BATTERY;
                openDevices++;
                break;

            case DEVICE_TYPE_DEADBOLT_SENSOR:
                nodeIDtoSN[learnedDeviceCount].curr_status = RF_DEADBOLT_SENSOR_OPEN | RF_DEADBOLT_SENSOR_LOW_BATTERY;
                lockDevices++;
                break;

            case DEVICE_TYPE_WINDOW_SHADE://enqueue shade learn ACK
                delay_shade_transmit = true;
                enqueue_shade_command(learnPktSN, WINDOW_SHADE_CMD_B2S_ACK, 0);
                break;

            default:
                break;
            }

            AddPairedDevice(NULL, mutex, nodeIDtoSN[learnedDeviceCount].SN,
                            currently_learning_device_type,
                            &nodeIDtoSN[learnedDeviceCount].id);

            char *idString = idToHexString(nodeIDtoSN[learnedDeviceCount].SN);

            if (idString)
                DbAddToBLCmdTable(NULL, mutex, BL_LEARN_SUCCESS, idString);

            learnedDeviceCount++;
        }

        // If timer is active this is learning during power up.
        // Learn initiated through button press is stopped by BL.
        // Will not track here.
        if (learnState == STATE_MACHINE_LEARN_DEVICE)
            state = STATE_MACHINE_LEARN_DEVICE;
        else
        {
            state = STATE_MACHINE_IDLE;
        }

        done = true;
        break;


    case STATE_MACHINE_UNLEARN_SUCCESS:
        // Unlearn-Remove from table
        if (!bUnlearnStatusModule)
        {

            if ((learnPktSN >> 19) == DEVICE_TYPE_WINDOW_SHADE)
                enqueue_shade_command(learnPktSN, WINDOW_SHADE_CMD_B2S_ACK, 0);//ack shade unlearn

            removeDeviceFromList(mutex, learnPktSN, 0);
            if (learnState == STATE_MACHINE_LEARN_DEVICE) // If BL initiated
            {
                //Inform BL about unlearn success
                DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, 0);
            }
        }


        bUnlearnStatusModule = false;
        if (learnState == STATE_MACHINE_LEARN_DEVICE)
        {
            state = STATE_MACHINE_LEARN_DEVICE;
        }
        else
        {
            state = STATE_MACHINE_IDLE;
        }
        break;

    case STATE_MACHINE_LEARN_ABORT:
        // Notified by BL
    {
        state = STATE_MACHINE_IDLE;
        // If there is a Status Module, send result
        SendInstallStatus(RF_STATUS_MODULE_LEARN_FAIL);
    }
    break;

    case STATE_MACHINE_LEARN_DEVICE:
        // Just entered state, kick-off UI
        Stop_GeneralFault((enum GENERAL_FAULTS)0xFF);
        done = true;
        break;


    case STATE_MACHINE_LEARN_STATUS_MODULE:
        // Just entered state, kick-off UI
        Stop_GeneralFault((enum GENERAL_FAULTS)0xFF);
        done = true;
        break;

    case STATE_MACHINE_CLEAR_MEMORY:
        Stop_GeneralFault((enum GENERAL_FAULTS)0xFF);

        openedCount = 0;
        unlockedCount = 0;

        // Set Closed/Locked for all learnedDevices
        for (i = 0; i < learnedDeviceCount; i++)
        {
            nodeIDtoSN[i].curr_status = 0;
            nodeIDtoSN[i].receiveTime = 0;
            nodeIDtoSN[i].supervisorMissed = 0;
        }

        generalFault[GENERAL_FAULT_LOS].active = 0;
        generalFault[GENERAL_FAULT_LOS].counter = 0;
        generalFault[GENERAL_FAULT_ZWAVE_ERR].active = 0;
        generalFault[GENERAL_FAULT_ZWAVE_ERR].counter = 0;

        state = STATE_MACHINE_IDLE;
        break;

    case STATE_MACHINE_FACTORY_DEFAULT:
        state = STATE_MACHINE_POR;
        break;

    case STATE_MACHINE_DIAG_DEVICE_TO_HUB:
        if (startDiag)
        {   // First time in state, beep number of learned devices
            Stop_GeneralFault((enum GENERAL_FAULTS)0xFF);
            // Start Diag time-out timer
            diagTimeStart = boomTicks;
            rf_pktCount = 0;
            first_pkt_time = 0;
            startDiag = false;
            TimerStop(s_IdleTimer);
        }

        // Look for a good RF receive
        if (rf_pktCount > 0)
        {
            // Good packet, turn on Green LED and start Beeper
            if (first_pkt_time == 0)
            {   // First pkt, start 1.6 second window
                first_pkt_time = boomTicks;
                TimerStart(s_IdleTimer, 1, 600, 0, 0);
            }
            // See if 1.8 seconds passed since 1st receive
            if (!TimerIsActive(s_IdleTimer))
            {
                if (rf_pktCount >= 6)//TBD
                {
                    //Inform BL of success
                    DbAddToBLCmdTable(NULL, mutex, BL_SENSOR_TEST_SUCCEEDED, 0);
                }
                else if (rf_pktCount < 3)//TBD
                {   // Not all pkts received, FAIL
                    //Inform BL of failure
                    DbAddToBLCmdTable(NULL, mutex, BL_SENSOR_TEST_FAILED, 0);
                }
                rf_pktCount = 0;
                first_pkt_time = 0;
            }
        }

        // Check for Diagnostic timeout
        if (force_diagExit)
        {
            //force_diagExit is set when the test button is pressed and held
            // for >0.8 sec
            force_diagExit = false;
            state = STATE_MACHINE_IDLE;
        }
        done = true;
        break;

    case STATE_MACHINE_DIAG_HUB_TO_ZWAVE:
        break;

    case STATE_MACHINE_FIRMWARE_UPGRADE:
        //TBD
        // For now, handled in bootloader
        //  Upgrade requires a hold-down of Learn button during power cycle
        state = STATE_MACHINE_IDLE;
        break;

    case STATE_MACHINE_LEARN_WAIT:
        break;

    case STATE_MACHINE_IDLE:
    {

        if (entering_new_state)
        {
            TimerStart(s_IdleTimer, IDLE_CHECK_TIMEOUT, 0, 0, 0);
        }
        //
        // Every 10 seconds, check for Supervisor for all Transmitters/Sensors
        //
        if (!TimerIsActive(s_IdleTimer))
        {   // Every 10 seconds, check the Supervisory state of all active virtual devices
            // See if LOS is starting
            if ((!generalFault[GENERAL_FAULT_LOS].active && LOS_state))// || err_interrupted) BG: I believe this is the cause of the constant beeping bug
            {   // Turn on LOS UI state
                Start_GeneralFault(GENERAL_FAULT_LOS);
            }
            // See if LOS has been resolved
            else if (generalFault[GENERAL_FAULT_LOS].active && !LOS_state)
            {   // Turn off LOS UI state
                Stop_GeneralFault(GENERAL_FAULT_LOS);
            }

            // See if ZWave Error UI
            //  note: LOS UI overrides ZWave Err UI

            if (!generalFault[GENERAL_FAULT_LOS].active)
            {
                // See if ZWAVE_ERR is starting
                if ((!generalFault[GENERAL_FAULT_ZWAVE_ERR].active && ZWave_err_state))// || err_interrupted)//BG: I'll comment this one out as well for good measure.
                {   // Turn on ZWave_err_state UI state
                    Start_GeneralFault(GENERAL_FAULT_ZWAVE_ERR);
                }
                // See if ZWave_err_state has been resolved
                else if (generalFault[GENERAL_FAULT_ZWAVE_ERR].active && !ZWave_err_state)
                {   // Turn off ZWave_err_state UI state
                    Stop_GeneralFault(GENERAL_FAULT_ZWAVE_ERR);
                }
            }
            TimerStart(s_IdleTimer, IDLE_CHECK_TIMEOUT, 0, 0, 0);
        }
        done = true;
    }
    break;//STATE_MACHINE_IDLE end

    default:
        break;
    }

    if (!done) {
        // Send mock event to myself to trigger the state machine
        struct Event evt;
        evt.event = SM_EVENT;
        EventQ_Send(&evt);
    }
}


void send_queued_packets(void)
{
    enum ShadeTxStates {
        NoShadeTx,
        DelayShadeTx,
        ShadeTx
    };

    static timer_t shadeTxTimer = 0;
    static enum ShadeTxStates shadeStates = NoShadeTx;

    if (shadeTxTimer == 0)
        TimerInit(&shadeTxTimer, TIMER_EVENT);

    if (delay_shade_transmit) {
        delay_shade_transmit = false;
        TimerStart(shadeTxTimer, 0, 500, 0, 0);
        shadeStates = DelayShadeTx;
    }

    if (TimerIsActive(shadeTxTimer) && shadeStates == DelayShadeTx)
        return;

    if (shadeStates == DelayShadeTx) {
        shadeStates = ShadeTx;
        TimerStop(shadeTxTimer);
        TimerStart(shadeTxTimer, 0, SHADE_RETRANSMIT_INTERVAL, 0, 0);
    }

    // Look for packets shade commands that need transmitting
    // Every 125ms check for a pending shade command
    if (!TimerIsActive(shadeTxTimer) && (shadeStates == ShadeTx)) {
        TimerStop(shadeTxTimer);
        pending_shade_command pending_command;
        pending_command.shade_serial = 0;
        getNext_shade_command(&pending_command);

        // If found, transmit the command
        if (pending_command.shade_serial != 0)
        {
            rf_bytes[ RF_IDX_SN_MSB ] = pending_command.shade_serial >> 16;
            rf_bytes[ RF_IDX_SN_MID ] = pending_command.shade_serial >>  8;
            rf_bytes[ RF_IDX_SN_LSB ] = pending_command.shade_serial;
            rf_bytes[ RF_IDX_DATA   ] = pending_command.shade_command;

            //Commands are either 1 or 2 bytes

            if ((pending_command.shade_command ==  WINDOW_SHADE_CMD_B2S_SET_SHADE_POS) ||
                (pending_command.shade_command == WINDOW_SHADE_CMD_B2S_SET_VENETIAN_ANGLE) ||
                (pending_command.shade_command == WINDOW_SHADE_CMD_B2S_GET_LIMIT)) {

                rf_bytes[ RF_IDX_DATA + 1 ] = pending_command.param;
                rf_bytes[ RF_IDX_LEN ] = 6;
            } else {
                rf_bytes[ RF_IDX_LEN ] = 5;
            }

            Ecolink_Tx_Shade();
            TimerStart(shadeTxTimer, 0, SHADE_RETRANSMIT_INTERVAL, 0, 0);

        } else {
            shadeStates = NoShadeTx;
            TimerStop(shadeTxTimer);
        }
    }
}

static void Options (int argc, char *argv[], int* port)
{
    int opt;

    while ((opt = getopt (argc, argv, "vp:")) != -1)
    {
        switch (opt)
        {
        case 'p':
            if (port) {
                *port = atoi(optarg);
            }
            break;

        default:
            break;
        }
    }
}

/******************************************************************************
 * Function:        main
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
int main(int argc, char *argv[])
{
    int port = -1;
    int i;
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

    resetOldBytes();
    setlinebuf(stdout);
    openlog("insynctive", LOG_PID|LOG_CONS, LOG_LOCAL0);
    PrintVersion();
    Options(argc, argv, &port);

    if (port < 0) {
        fprintf(stderr, "Invalid or missing Insynctive interface port\n");
        exit(-1);
    }

    // Initialize the radio
    radio_init(port);
    EventQ_Open();

    boomSN = DBReadBoomSerialNum(NULL, &mutex);

    int numDevices = DbGetTotalDevices(NULL, &mutex) - 1;

    activeStatusModules = 0;

    if (numDevices > 0) {

        DbReadPairedDevices(NULL, &mutex);

        for (i = 0; i < learnedDeviceCount; i++) {

            nodeIDtoSN[i].receiveTime = 0;
            nodeIDtoSN[i].supervisorMissed = false;
            nodeIDtoSN[i].curr_status = 0;

            switch(nodeIDtoSN[i].device_type) {

            case DEVICE_TYPE_TILT_SENSOR:
            case DEVICE_TYPE_DOOR_WINDOW_SENSOR:
                nodeIDtoSN[i].curr_status = RF_DOOR_WINDOW_OPEN;
                openedCount++;
                openDevices++;
                break;

            case DEVICE_TYPE_DEADBOLT_SENSOR:
                nodeIDtoSN[i].curr_status = RF_DEADBOLT_SENSOR_UNLOCKED;
                unlockedCount++;
                lockDevices++;
                break;

            default:
                break;
            }
        }

        UpdateStartupValues(NULL, &mutex);
    } else {

        for (i = 0; i< MAX_DEVICES; ++i)
            memset(nodeIDtoSN + i, 0, sizeof(struct NODE_ID_TO_SN));

        // Clear out Status Modules
        for (i = 0; i < MAX_STATUS_MODULES; i++) {
            statusModule[i].SN = 0;
            statusModule[i].curr_status = 0;
        }

        activeStatusModules = 0;
        openDevices = 0;
        lockDevices = 0;
    }

    // Break down BoomBridge SN for code convenience
    boomSN_msb = boomSN >> 16;
    boomSN_mid = boomSN >> 8;
    boomSN_lsb = boomSN >> 0;

    state = STATE_MACHINE_POR;

    shade_command_queue_init();

    for (i = 0; i < TOTAL_GENERAL_FAULTS; i++) {
        generalFault[i].active = false;
        generalFault[i].counter = 0;
    }

    CreateThreads(&mutex);

    struct Event evt;
    unsigned char* buf;

    last_state = STATE_MACHINE_POR;

    TimerInit(&s_RxTimer, TIMER_EVENT);

    StartRssiTimer();

    while(1) {
        step_bridgeStateMachine(&mutex);

        memset(&evt, 0, sizeof(evt));

        EventQ_Receive(&evt);

        boomTicks = clock();

        buf = 0;

        if (evt.event == UART_EVENT) {
            buf = evt.data.num;
        } else if (evt.event == REG_EVENT) {
            if (evt.data.num[0] == 0x24) {
                rssi_val = evt.data.num[1];
            }
        } else if (evt.event == DB_EVENT) {
            HandleDBCommands(&mutex, &evt.data.dbEvent);
        } else if (evt.event == TIMER_EVENT) {
            if (s_RxTimer && TimerIsActive(s_RxTimer)) {
                radio_data_in(RSSI_REG);
            }
        } else if (evt.event == STATUS_MODULE_TX_EVENT) {
            CheckStatusModuleTimeout();
            Retransmit();
        } else if (evt.event == RD_VERSION_EVENT) {
            syslog(LOG_INFO, "Insynctive PIC Version %d.%d.%d",
                   evt.data.num[0], evt.data.num[1], evt.data.num[2]);
        }

        // Look for RF receive
        if (Ecolink_Rx(buf)) {
            handleIncomingRFPacket(&mutex, NULL);
        }

        if (state != last_state) {
            entering_new_state = true;
            last_state = state;
        } else {
            entering_new_state = false;
        }

        send_queued_packets();
    }

}

void save_command_and_param(uint32_t serial_number, uint8_t cmd, uint8_t param)
{
    int i = 0;

    /* Save the last command sent to a shade */

    /* Note: the command should really only saved if it gets acked
             from the shade */

    for (; i < learnedDeviceCount; ++i) {
        if (nodeIDtoSN[i].SN == serial_number) {
            nodeIDtoSN[i].last_insynctive_command_sent = cmd;
            nodeIDtoSN[i].last_insynctive_param_sent = param;
            i = learnedDeviceCount;
        }
    }
}

void
setStatusByte(void)
{   // Prepare RF packet for Status Module

    int i = 0;

    for (i = 0; i < TOTAL_GENERAL_FAULTS; i++)
        if (generalFault[i].active)
            break;

    int condition =
        ((unlockedCount != 0) ? 1 : 0) +
        ((lockDevices > 0) ? (1 << 1) : 0) +
        ((openedCount != 0) ? (1 << 2) : 0) +
        ((openDevices > 0) ? (1 << 3): 0);

    switch(condition)
    {
    case 0b1010: //((openDevices > 0 && openedCount == 0) && (lockDevices > 0 && unlockedCount == 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_ALL_CLOSED_ALL_LOCKED;
        break;

    case 0b1011: //((openDevices > 0 && openedCount == 0) && (lockDevices > 0 && unlockedCount != 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_ALL_CLOSED_SOME_UNLOCKED;
        break;

    case 0b1110: //((openDevices > 0 && openedCount != 0) && (lockDevices > 0 && unlockedCount == 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_SOME_OPENED_ALL_LOCKED;
        break;

    case 0b1111: //if ((openDevices > 0 && openedCount != 0) && (lockDevices > 0 && unlockedCount != 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_SOME_OPENED_SOME_UNLOCKED;
        break;

    case 0b0111:
    case 0b0011: // ((openDevices == 0                  ) && (lockDevices > 0 && unlockedCount != 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_NO_CLOSED_SOME_UNLOCKED;
        break;

    case 0b0010:
    case 0b0110: // ((openDevices == 0                  ) && (lockDevices > 0 && unlockedCount == 0))
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_NO_CLOSED_ALL_LOCKED;
        break;

    case 0b1100:
    case 0b1101: // ((openDevices > 0 && openedCount != 0) && (lockDevices == 0)                    )
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_SOME_OPENED_NO_LOCKED;
        break;

    case 0b1001:
    case 0b1000: //((openDevices > 0 && openedCount == 0) && (lockDevices == 0)                    )
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_ALL_CLOSED_NO_LOCKED;
        break;

    case 0b0000:
    case 0b0001:
    case 0b0100:
    case 0b0101:
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_NO_CLOSED_NO_LOCKED;
        break;

    default:
        rf_bytes[RF_IDX_DATA] = RF_STATUS_MODULE_NO_CLOSED_NO_LOCKED;

    }

    if (i != TOTAL_GENERAL_FAULTS) {
        // General Fault found, notify Status Module
        rf_bytes[RF_IDX_DATA] |= RF_STATUS_MODULE_GENERAL_FAULT_MASK;
    }
}


void notifyStatusModule(void)
{
    unsigned int i;

    /* If any active Status Modules in Status mode, send them the
     * updated status */

    // Look for active Status module
    for (i = 0; i < activeStatusModules; i++) {
        ResetStatusModuleStatus(i);
        if (statusModule[i].curr_status == RF_STATUS_MODULE_STATUS_MODE) {
            rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
            rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
            rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
            setStatusByte();
            Ecolink_Tx_wait_ACK(6, statusModule[i].SN);
        }
    }
}

void CreateThreads(pthread_mutex_t *mutex)
{
#define STACK_SIZE 2048
    pthread_attr_t threadAttr;

    (void)pthread_attr_init(&threadAttr);
    (void)pthread_attr_setstacksize(&threadAttr, STACK_SIZE);
    (void)pthread_attr_setdetachstate (&threadAttr, PTHREAD_CREATE_DETACHED);

    pthread_t threadId;

    /* UART Thread */
    if (pthread_create(&threadId, &threadAttr, &RadioThread, (void*)mutex) != 0) {
        syslog(LOG_INFO, "pthread_create ZwaveUartThread failed \n");
        exit(EXIT_FAILURE);
    }

    /* DB Monitor Thread */
    if (pthread_create(&threadId, &threadAttr, &DBThread, (void *)mutex) != 0) {
        syslog(LOG_INFO, "pthread_create DBThread failed\n");
        exit(EXIT_FAILURE);
    }

    /* /\* Change monitoring thread *\/ */
    /* if (pthread_create(&threadId, &threadAttr, &ChangeThread, (void *)mutex) != 0) { */
    /*     syslog(LOG_INFO, "pthread_create DBThread failed\n"); */
    /*     exit(EXIT_FAILURE); */
    /* } */
}


void Start_GeneralFault(enum GENERAL_FAULTS type_generalFault)
{
    generalFault[type_generalFault].active = true;
}

void Stop_GeneralFault(enum GENERAL_FAULTS fault_type)
{
    if (fault_type < TOTAL_GENERAL_FAULTS) {
        if (generalFault[ fault_type ].active) {

            // BG This is added by me here, and I am replacing
            // everywhere this flag was manually set to use this
            // function instead to be sure the buzzer gets turned off
            generalFault[fault_type].active = false;
        }
    }
}

void Handle_GeneralFault(void)
{
    syslog(LOG_ERR, "Handle_GeneralFault() invoked");

    return;
}

bool virtualReceive(unsigned long pkt_sn)
{
    for (unsigned int i = 0; i < learnedDeviceCount; i++)
        if (nodeIDtoSN[i].SN == pkt_sn) {
            myNodeID_idx = i;
            return true;
        }

    return false;
}


void removeDeviceFromList(pthread_mutex_t *mutex, unsigned long deviceSN,
                          unsigned char id)
{

    unsigned int i;

    for (i = 0; i < learnedDeviceCount; i++) {
        if (deviceSN == nodeIDtoSN[i].SN) {
            if ((id == 0) ||
                ((id != 0) && (id == nodeIDtoSN[i].id))) {

                // Adjust Status Module info (Open/Lock device count,
                //                            Open/Unlock count)

                switch (nodeIDtoSN[i].device_type) {
                case DEVICE_TYPE_TILT_SENSOR:
                case DEVICE_TYPE_DOOR_WINDOW_SENSOR:
                    openDevices--;
                    if (nodeIDtoSN[i].curr_status & RF_DOOR_WINDOW_OPEN)
                        openedCount--;
                    if (nodeIDtoSN[i].curr_status & RF_DOOR_WINDOW_TAMPERED)
                        openedCount--;
                    break;
                case DEVICE_TYPE_DEADBOLT_SENSOR:
                    lockDevices--;
                    if (nodeIDtoSN[i].curr_status & RF_DEADBOLT_SENSOR_UNLOCKED)
                        unlockedCount--;
                    if (nodeIDtoSN[i].curr_status & RF_DEADBOLT_SENSOR_TAMPERED)
                        unlockedCount--;
                    break;
                default:
                    break;
                }

                RemovePairedDevice(NULL, mutex, nodeIDtoSN[i].id);

                // Adjust internal table
                //  (not circular buffer, so push everything up)

                for (; i < learnedDeviceCount; i++) {
                    nodeIDtoSN[i].id = nodeIDtoSN[i+1].id;
                    nodeIDtoSN[i].SN = nodeIDtoSN[i+1].SN;
                    nodeIDtoSN[i].device_type = nodeIDtoSN[i+1].device_type;
                    nodeIDtoSN[i].curr_status = nodeIDtoSN[i+1].curr_status;
                    nodeIDtoSN[i].receiveTime = nodeIDtoSN[i+1].receiveTime;
                    nodeIDtoSN[i].supervisorMissed = nodeIDtoSN[i+1].supervisorMissed;
                    nodeIDtoSN[i].last_insynctive_command_sent = nodeIDtoSN[i+1].last_insynctive_command_sent;
                    nodeIDtoSN[i].last_insynctive_param_sent = nodeIDtoSN[i+1].last_insynctive_param_sent;
                }

                learnedDeviceCount--;

                break;
            }
        }
    }

    // Notify Status module
    notifyStatusModule();
}


bool handle_shade_to_zwave(pthread_mutex_t *mutex)
{

    switch(rcvd_bytes[ RF_IDX_DATA ])
    {
    case WINDOW_SHADE_CMD_S2B_ACK:
    {
        erase_remaining_sent_command_queued_for_shade(pktSN);

        for (int i = 0; i < MAX_DEVICES; ++i)
        {
            if (nodeIDtoSN[ i ].SN == pktSN)
            {
                nodeIDtoSN[ i ].receiveTime = boomTicks;

                if (WINDOW_SHADE_CMD_B2S_SET_SHADE_POS == nodeIDtoSN[ i ].last_insynctive_command_sent)
                {
                    if (nodeIDtoSN[ i ].last_insynctive_param_sent == BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_UPPER)
                    {
                        nodeIDtoSN[ i ].curr_status = 0x00;
                    }
                    else if (nodeIDtoSN[ i ].last_insynctive_param_sent == BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_LOWER)
                    {
                        nodeIDtoSN[ i ].curr_status = 0x64;
                    }
                    else if (nodeIDtoSN[ i ].last_insynctive_param_sent >= 0 && nodeIDtoSN[ i ].last_insynctive_param_sent <= 0x64)
                    {
                        nodeIDtoSN[ i ].curr_status = nodeIDtoSN[ i ].last_insynctive_param_sent;
                    }
                    UpdateStatus(NULL, mutex, nodeIDtoSN[i].id, nodeIDtoSN[i].device_type,
                                 POSITION, nodeIDtoSN[i].curr_status, &
                                 nodeIDtoSN[i].receiveTime, false);
                }
                i = MAX_DEVICES;
            }
        }
    }
    break;

    case WINDOW_SHADE_CMD_S2B_REPORT_SHADE_POSITION:
    {
        unsigned char level = rcvd_bytes[ RF_IDX_DATA + 1 ];

        switch(level)
        {
        case BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_LOWER://lower soft limit
            level = 0x64;
            break;

        case BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_UPPER://upper soft limit
            level = 0x00;
            break;

        case BRIDGE_TO_SHADE_PARAM_SAVED_POSITION://saved position
        case BRIDGE_TO_SHADE_PARAM_HALT_MOVEMENT:
        default:
            if (level >= 100)
            {
                level = 0x64;
            }
        }

        nodeIDtoSN[myNodeID_idx].curr_status = level;
        nodeIDtoSN[myNodeID_idx].receiveTime = boomTicks;

        UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                     nodeIDtoSN[myNodeID_idx].device_type, POSITION,
                     nodeIDtoSN[myNodeID_idx].curr_status,
                     &nodeIDtoSN[myNodeID_idx].receiveTime, false);
        UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                     nodeIDtoSN[myNodeID_idx].device_type, SHADE_ACK,
                     true, &nodeIDtoSN[myNodeID_idx].receiveTime, false);
        erase_all_of_same_commands_for_shade(pktSN, WINDOW_SHADE_CMD_B2S_GET_SHADE_POS);
    }
    break;

    case WINDOW_SHADE_CMD_S2B_REPORT_VENETIAN_ANGLE:
    {
        UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                     nodeIDtoSN[myNodeID_idx].device_type, ANGLE,
                     rcvd_bytes[RF_IDX_DATA + 1], &boomTicks, false);
        erase_all_of_same_commands_for_shade(pktSN, WINDOW_SHADE_CMD_B2S_GET_VENETIAN_ANGLE);
    }
    break;

    case WINDOW_SHADE_CMD_S2B_REPORT_BATTERY_LEVEL:
    {
        if (rcvd_bytes[RF_IDX_DATA + 1] >=0 && rcvd_bytes[RF_IDX_DATA + 1] <= 100)
        {

            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                         nodeIDtoSN[myNodeID_idx].device_type, BATT_LEVEL,
                         rcvd_bytes[RF_IDX_DATA + 1], &boomTicks, false);
        }
        else
        {
            UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                         nodeIDtoSN[myNodeID_idx].device_type, BATT_LEVEL,
                         0, &boomTicks, false);
        }
        erase_all_of_same_commands_for_shade(pktSN, WINDOW_SHADE_CMD_B2S_GET_BATTERY_LEVEL);
    }
    break;

    case WINDOW_SHADE_CMD_S2B_REPORT_LIMIT_POSITION:
    {
        if (WINDOW_SHADE_CMD_B2S_GET_LIMIT == nodeIDtoSN[ myNodeID_idx].last_insynctive_command_sent)
        {
            enum ATTRIBUTE attrib;
            bool attribSet = false;

            if (nodeIDtoSN[ myNodeID_idx ].last_insynctive_param_sent == BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_UPPER)
            {
                attrib = UPPER_SOFT_LIMIT;
                attribSet = true;
            }
            else if (nodeIDtoSN[ myNodeID_idx ].last_insynctive_param_sent == BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_LOWER)
            {
                attrib = LOWER_SOFT_LIMIT;
                attribSet = true;
            }
            else if (nodeIDtoSN[myNodeID_idx].last_insynctive_param_sent >= 0 && nodeIDtoSN[myNodeID_idx].last_insynctive_param_sent <= 0x64)
            {
                attrib = SAVED_POSITION;
                attribSet = true;
            }

            if (attribSet)
                UpdateStatus(NULL, mutex, nodeIDtoSN[myNodeID_idx].id,
                             nodeIDtoSN[myNodeID_idx].device_type, attrib,
                             rcvd_bytes[RF_IDX_DATA + 1],
                             &nodeIDtoSN[myNodeID_idx].receiveTime, false);
        }
        erase_all_of_same_commands_for_shade(pktSN, WINDOW_SHADE_CMD_B2S_GET_LIMIT);
    }
    break;

    default:
        break;

    }
    return true;

}

void removeStatusModuleFromList(unsigned long statusSN)
{
    for (int i = 0; i < (activeStatusModules - 1); i++) {
        if (statusSN == statusModule[i].SN) {
            for (; i < activeStatusModules; i++) {
                statusModule[i].SN = statusModule[i+1].SN;
                statusModule[i].curr_status = statusModule[i+1].curr_status;
            }

            break;
        }
    }

    activeStatusModules--;
    bUnlearnStatusModule = true;
}


void AddToNodeIDTable(struct NODE_ID_TO_SN *n)
{
    if (n->id == 0) {
        syslog(LOG_INFO, "AddToNodeIDTable(): Id cannot be 0");
        return;
    }

    if (haveSpaceForAnotherDevice()) {
        nodeIDtoSN[learnedDeviceCount].id = n->id;
        nodeIDtoSN[learnedDeviceCount].device_type = n->device_type;
        nodeIDtoSN[learnedDeviceCount].SN = n->SN;

        ++learnedDeviceCount;
    }
}

void AddStatusModule(unsigned char id, unsigned long   SN)
{
    if (id == 0)
    {
        syslog(LOG_INFO, "AddStatusModule(): Id cannot be 0");
        return;
    }

    if (haveSpaceForAnotherStatusModule()) {
        statusModule[activeStatusModules].id = id;
        statusModule[activeStatusModules].SN = SN;
        ++activeStatusModules;
    }
}

int IsDeviceLearnt(unsigned char id)
{
    int i = 0;

    for (i = 0; i < learnedDeviceCount; i++)
        if (nodeIDtoSN[i].id == id)
            break;

    if (i == learnedDeviceCount)
        if (false == isStatusModule(id, &i))
            i = 0;

    return i;
}


void HandleShadeAttribute(const struct DbEvent *evt, unsigned long SN)
{
    delay_shade_transmit = true;
    unsigned long eventValue = strtoul(evt->value, NULL, 0);

    switch(evt->attrib) {
    case BATT_LEVEL:
        enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_BATTERY_LEVEL, 0);
        break;

    case POSITION:
        if (evt->cmd == SET) {
            enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_SET_SHADE_POS, (uint8_t)eventValue);
            shadeReportTime = boomTicks;
            checkShade = true;
        } else {
            enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_SHADE_POS, 0);
        }
        break;

    case ANGLE:
        if (evt->cmd == SET)
            enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_SET_VENETIAN_ANGLE, (uint8_t)eventValue);
        else
            enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_VENETIAN_ANGLE, 0);
        break;

    case LOWER_SOFT_LIMIT:
        enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_LIMIT, BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_LOWER);
        break;

    case UPPER_SOFT_LIMIT:
        enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_LIMIT, BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_UPPER);
        break;

    case SAVED_POSITION:
        enqueue_shade_command(SN, WINDOW_SHADE_CMD_B2S_GET_LIMIT, BRIDGE_TO_SHADE_PARAM_SAVED_POSITION);
        break;

    default:
        break;
    }
}

void HandleDBCommands(pthread_mutex_t *mutex, const struct DbEvent *evt)
{
    int i = IsDeviceLearnt(evt->id);
    unsigned long eventValue = strtoul(evt->value, NULL, 0);


    switch(evt->cmd) {
    case SET:
    case GET:
        if ((i < learnedDeviceCount) &&
            (nodeIDtoSN[i].device_type == DEVICE_TYPE_WINDOW_SHADE)) {
            HandleShadeAttribute(evt, nodeIDtoSN[i].SN);
        }
        break;

    case ENTER_LEARN:
        if (state == STATE_MACHINE_IDLE) {
            state = STATE_MACHINE_LEARN_DEVICE;
            learnState = state;
            learnPktSN = 0xfe;
        }
        break;

    case EXIT_LEARN:
        if ((state == STATE_MACHINE_LEARN_DEVICE) ||
            (state == STATE_MACHINE_LEARN_STATUS_MODULE)) {
            state = STATE_MACHINE_IDLE;
            learnState = STATE_MACHINE_IDLE;
        }
        break;

    case FORGET_DEVICE:
        if (state == STATE_MACHINE_IDLE) {
            int statusInd;
            if (isStatusModule(evt->id, &statusInd)) {
                RemovePairedDevice(NULL, mutex, statusModule[statusInd].id);
                removeStatusModuleFromList(statusModule[statusInd].SN);
            } else {
                removeDeviceFromList(mutex, nodeIDtoSN[i].SN, 0);
            }

            DbAddToBLCmdTable(NULL, mutex, BL_UNLEARN_SUCCESS, 0);
        }
        break;

    case ENTER_STATUS_MODULE_LEARN:
        if (state == STATE_MACHINE_IDLE) {
            state = STATE_MACHINE_LEARN_STATUS_MODULE;
            learnState = state;
        }

        break;

    case ENTER_ALL_BUT_SM:

        if (state == STATE_MACHINE_IDLE)
            LearnAllButStatusModules = true;

        break;

    case ZWAVE_LEARN_FAILED:
        SendInstallStatus(RF_STATUS_MODULE_LEARN_FAIL);
        break;

    case ZWAVE_LEARN_SUCCESS:
        Start = false;
        if (LearnAllButStatusModules) {
            //Device successfully added from the z-wave network
            learnPktSN = 0xfe;
        }
        SendInstallStatus(RF_STATUS_MODULE_LEARN_SUCCESS);
        break;

    case ZWAVE_UNLEARN_SUCCESS:
        Start = false;
        if (LearnAllButStatusModules) {
            //Device successfully added or removed from the z-wave network
            learnPktSN = 0xfe;
        }
        break;

    case CLEAR_MEMORY:
        state = STATE_MACHINE_CLEAR_MEMORY;
        break;

    case ENTER_DIAG_MODE:
        if (state == STATE_MACHINE_IDLE) {
            startDiag = true;
            state = STATE_MACHINE_DIAG_DEVICE_TO_HUB;
        }
        break;

    case EXIT_DIAG_MODE:
        if (state == STATE_MACHINE_DIAG_DEVICE_TO_HUB)
            force_diagExit = true;
        break;

    case ZWAVE_ERROR:
        if (eventValue)
            ZWave_err_state = true;
        else
            ZWave_err_state = false;
        break;

    case SUPERVISORY_FAULT:
        if (eventValue)
            LOS_state = true;
        else
            LOS_state = false;

        if (i < learnedDeviceCount) {
            //Sets the device into superviory fault
            nodeIDtoSN[i].super_fault_status = true;
            nodeIDtoSN[i].receiveTime = boomTicks;
            //Writes to data the device is in superviory fault
            UpdateStatus(NULL, mutex, nodeIDtoSN[i].id, nodeIDtoSN[i].device_type,
                         IS_SUPER_FAULT, true, &nodeIDtoSN[i].receiveTime, false);
        }
        break;

    case LEARN_NAK:
    {
        unsigned long serialNumber;

        sscanf(evt->value, "%li", &serialNumber);
        removeDeviceFromList(mutex, serialNumber, 0);

        pktSN = 0;
        learnPktSN = 0;
    }
    break;

    case UNLEARN_ACK:
        break;

    case UNLEARN_NAK:
        break;

    default:
        break;
    }
}


void SendInstallStatus(unsigned char status)
{
    for (int i = 0; i  < activeStatusModules; i++) {

        // Tranmsit to Status Module, retry until ACK

        if (statusModule[i].curr_status == RF_STATUS_MODULE_INSTALL_MODE) {

            // Give time to Status Module to switch to receive mode
            delay_ms(5);

            rf_bytes[RF_IDX_SN_MSB] = boomSN_msb;
            rf_bytes[RF_IDX_SN_MID] = boomSN_mid;
            rf_bytes[RF_IDX_SN_LSB] = boomSN_lsb;
            rf_bytes[RF_IDX_DATA] = status;
            Ecolink_Tx_wait_ACK(6, statusModule[i].SN);
        }
    }
}


void PrintVersion()
{
    syslog(LOG_INFO, "Insynctive - v%d.%d.%d\n", MAJOR_VERSION, MINOR_VERSION, BUILD_VERSION);
}


void ResetStatusModuleStatus(int i)
{
    /* Look for any Status Module mode timeouts (corner case: Status
     * module goes into Status/Install/Test modes and doesn't ever end
     * mode (pwr failure) */

    if ((statusModule[i].curr_status == RF_STATUS_MODULE_TEST_MODE)   ||
        (statusModule[i].curr_status == RF_STATUS_MODULE_STATUS_MODE) ||
        (statusModule[i].curr_status == RF_STATUS_MODULE_INSTALL_MODE))

        if (s_StatusModuleTimer && !TimerIsActive(s_StatusModuleTimer))
            statusModule[i].curr_status = 0;
}

void CheckStatusModuleTimeout()
{
    for (int i = 0; i < activeStatusModules; i++)
        ResetStatusModuleStatus(i);
}

bool isStatusModule(unsigned char id, int *index)
{
    for (*index = 0; *index < activeStatusModules; (*index)++)
        if (statusModule[*index].id == id)
            return true;

    return false;

}

void StartRssiTimer()
{
#define RSSI_TIME_INTERVAL 1

    TimerStart(s_RxTimer, RSSI_TIME_INTERVAL, 0, RSSI_TIME_INTERVAL, 0);
}
