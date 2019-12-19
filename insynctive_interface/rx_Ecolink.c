/** I N C L U D E S **********************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <syslog.h>
#include <string.h>
#include <stdio.h>
#include "ESW1032P-01.h"
#include "Timer.h"


/** E X T E R N A L S **********************************************************/
// RF receive variables
extern unsigned short crcWord;
extern unsigned char crcByte;
extern BYTE txPktQueueCount;
extern unsigned char rcvd_bytes[MAX_RCVD_BYTES];
extern unsigned long pktSN;

unsigned char rf_bytes[66];
static struct TX_PKT_BUFFER txPktBuffer[MAX_TX_QUEUE];
unsigned char rcvd_idx = 0;
unsigned char rf_idx = 0;
static timer_t s_TxStatusModule = 0;

BOOL decrypt_packet(unsigned char rf_bytes[]);

/** D E F I N E S ************************************************************/
#define TIMER_TICK_IN_NS    100

#define WHOLE_BIT_TIME_75
#define WHOLE_BIT_TIMR_1_25
#define HALF_BIT_TIME_75
#define HALF_BIT_TIME_1_25

#define DEGLITCH    1
#define MAX_GLITCH_IN_NS    (50000/TIMER_TICK_IN_NS)

#define STATUS_MODULE_TX_TIMEOUT_S (1) //1.5ms
#define STATUS_MODULE_TX_TIMEOUT_MS (50) //1.5ms

#define SmartSync_POLY 0x85

static void DecTxPktQueueCount(int i);

/************* SmartSync_CRC8_byte *******************
 *   Implementing George's CRC calculation in C for SmartSync's RF CRC
 *   @param  uint8_t crcVal  This iteration of CRC calculations will use this value to start out
 *   @param  uint8_t data    This is the data that will generate the resulting CRC
 *   @return uint8_t crcVal  The resulting CRC
 *****************************************************/
uint8_t SmartSync_CRC8_byte( uint8_t crcVal, uint8_t data ){

    uint8_t carryBit = 0;
    uint8_t bitMask = 0x80;

    for (; bitMask != 0; bitMask >>= 1){

        carryBit = crcVal & 0x80;

        crcVal <<= 1;

        if ((bitMask & data) != 0) {

            // Bit is a 1

            if (carryBit == 0) {

                crcVal ^= SmartSync_POLY;

            }

        } else {

            // Bit is a 0

            if (carryBit) {

                // Note: carryBit is either a 0 (false) or an 0x80
                //       which is a logical (true) since it's just NOT
                //       zero

                crcVal ^= SmartSync_POLY;

            }
        }
    }

    return crcVal;
}

/************* SmartSync_CRC8_byteBuffer *******************
 *   Implementing George's CRC calculation in C for SmartSync's RF CRC
 *   @param  uint8_t* byteBuffer  A pointer to an array of bytes that the crc will iterate through to calculate the CRC
 *   @param  uint8_t bufferLength The size of the array.
 *   @return uint8_t The result of the CRC calculation
 ********************************************************/
uint8_t SmartSync_CRC8_byteBuffer(const uint8_t* byteBuffer, const uint8_t bufferLength){

    uint8_t crcResult = 0;
    uint8_t i = 0;

    for (; i < bufferLength ; ++i )
        crcResult = SmartSync_CRC8_byte(crcResult, byteBuffer[i]);

    return crcResult;
}

/******************* receivedValidInsynctivPacket *************************
 * Checks to see if the buffer contains a valid packet
 * @param   uint8_t*    workingBuffer   buffer to check for insynctive packet
 * @param   bool    returns TRUE if the buffer contains a valid Insynctive packet
 **************************************************************************/
bool receivedValidInsynctivPacket(uint8_t* workingBuffer){

    uint8_t calculatedCRC = SmartSync_CRC8_byteBuffer((workingBuffer+1), workingBuffer[0] - 1);

    if (calculatedCRC == workingBuffer[workingBuffer[0]])
        return true;

    return false;
}

//BG: I am attempting to fix this to be able to receive variable length packets
unsigned char  Ecolink_Rx(unsigned char* buf)
{
    int i;
    BYTE pkt_len;
    unsigned long target_sn;

#define RCV_LEN  70

    if (buf) {

        int bufPtr = 0;

        do {
            //BG: So why are there two buffers rcvd_butes and rf_bytes?
            pkt_len = rf_bytes[rf_idx] = buf[bufPtr++];
            rf_idx++;
            rcvd_bytes[rcvd_idx] = pkt_len;
            rcvd_idx++;
            // Get rest of pkt after length byte

            for (i = 1; i <= pkt_len && i < MAX_RCVD_BYTES; i++) {
                // pkt_len
                rcvd_bytes[rcvd_idx] = rf_bytes[rf_idx] = buf[bufPtr++];
                rf_idx++;
                rcvd_idx++;
            }

            rcvd_idx = 0;
            rf_idx = 0;

            //BG: packets too long should not have crc calculated on them
            if (pkt_len < MAX_RCVD_BYTES) {
                if (receivedValidInsynctivPacket(rcvd_bytes)) {
                    // Look for an expected ACK
                    if ((txPktQueueCount > 0) &&
                        (rcvd_bytes[RF_IDX_DATA] == RF_STATUS_MODULE_ACK)) {

                        target_sn = (((unsigned long)rf_bytes[RF_IDX_SN_MSB] << 16) |
                                     ((unsigned long)rf_bytes[RF_IDX_SN_MID] << 8)  |
                                     ((unsigned long)rf_bytes[RF_IDX_SN_LSB] << 0));

                        for (i = 0; i < txPktQueueCount; i++) {
                            if (target_sn == txPktBuffer[i].SN) {

                                //Expected ACK received, pull out of buffer
                                DecTxPktQueueCount(i);
                                HandleEndOfTx();

                                //No need to inform State Machine of
                                //this background occurence
                                return false;
                            }
                        }
                    }

                    return true;
                } else if (pkt_len == ENCRYPTED_PACKET_LENGTH) {
                    return decrypt_packet(rf_bytes);//See if packet is encrypted
                } else
                    return false;
            }

        } while(0);
    } else {

        return false;
    }

    return false;
}

void Retransmit()
{
    // Look for any re-Tx in Tx Queue
    if (txPktQueueCount > 0) {
        for (int i = 0; i < txPktQueueCount; i++) {
            if (!TimerIsActive(s_TxStatusModule) &&
                (txPktBuffer[i].retries != 0)) {

                printf("---Resend status mod---\n");

                txPktBuffer[i].tx_ack_timer = TickGet();
                txPktBuffer[i].retries--;
                memcpy(rf_bytes, txPktBuffer[i].rfData, RF_IDX_PKT_SIZE);
                Ecolink_Tx(6);

                if (txPktBuffer[i].retries == 0) {
                    DecTxPktQueueCount(i);
                }

                TimerStart(s_TxStatusModule, STATUS_MODULE_TX_TIMEOUT_S, STATUS_MODULE_TX_TIMEOUT_MS, 0, 0);
            }
        }
    }
}

void DecTxPktQueueCount(int i)
{
    --txPktQueueCount;

    for (; i < txPktQueueCount; i++) {
        // Move queue up
        txPktBuffer[i] = txPktBuffer[i+1];
    }
}


void Ecolink_Tx(BYTE pkt_size)//BG: alright, I'm going to temporarily fix this before rewriting it completely
{
    // Doesn't include length byte
    rf_bytes[RF_IDX_LEN] = pkt_size-1;

    //BG: the user of Ecolink_tx already has to access the rf_bytes
    //array to call this function so why do we have a parameter here
    //for the length?

    rf_bytes[pkt_size-1] = SmartSync_CRC8_byteBuffer((const uint8_t*)rf_bytes + 1, rf_bytes[RF_IDX_LEN] - 1);

    // Tranmit the packet and return to Receive Mode
    radio_transmit_packet(rf_bytes, pkt_size);
}

void Ecolink_Tx_Shade( void )
{
    rf_bytes[rf_bytes[RF_IDX_LEN]] = SmartSync_CRC8_byteBuffer(rf_bytes + 1, rf_bytes[RF_IDX_LEN] - 1 );

    // Tranmit the packet and return to Receive Mode
    radio_transmit_packet(rf_bytes, rf_bytes[RF_IDX_LEN] + 1);

}


/* Transmit pkt, wait up to 50ms for ACK. retry 5 times note this will
 * delay for up to 250ms */
void Ecolink_Tx_wait_ACK(BYTE pkt_size, unsigned long target_sn)
{
    /* Transmit an Ecolink RF pkt of size "pkt_size" */
    unsigned char i;

    if (rf_bytes == 0)
        return;

    if (s_TxStatusModule == 0)
        TimerInit(&s_TxStatusModule, STATUS_MODULE_TX_EVENT);

    prepPkt(pkt_size);

    /* If there is room in the tx Pkt Queue, put this Tx pkt in */
    if (txPktQueueCount < MAX_TX_QUEUE) {

        for (i = 0; i < RF_IDX_PKT_SIZE; i++)
            txPktBuffer[txPktQueueCount].rfData[i] = rf_bytes[i];

        txPktBuffer[txPktQueueCount].retries = 5;
        txPktBuffer[txPktQueueCount].SN = target_sn;
        txPktQueueCount++;

    }

    /* Send packet */
    radio_transmit_packet(rf_bytes, pkt_size);

    if (!TimerIsActive(s_TxStatusModule))
        TimerStart(s_TxStatusModule, STATUS_MODULE_TX_TIMEOUT_S,
                   STATUS_MODULE_TX_TIMEOUT_MS, 0, 0);
}

void prepPkt(BYTE pkt_size)
{
    BYTE i, j;
    BYTE crc_bit;
    BYTE data_bit;

    rf_bytes[RF_IDX_LEN] = pkt_size-1;//doesn't include length byte

    // Calculate CRC-8
    // Don't include the length byte in CRC8 calculation
    for (i = RF_IDX_SN_MSB, crcByte = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (crcByte & 0x80)
                crc_bit = 1;
            else
                crc_bit = 0;
            crcByte <<= 1;

            if (rf_bytes[i] & (0x80 >> j))
                data_bit = 1;
            else
                data_bit = 0;

            if (data_bit == 1 && crc_bit == 0)
                crcByte ^= 0x85;
            else if (data_bit == 0 && crc_bit != 0)
                crcByte ^= 0x85;
        }
    }
    rf_bytes[RF_IDX_CRC] = crcByte;
}

BYTE decrypt_mask[8][7] =
{
    {0x29, 0xC8, 0x39, 0x89, 0x20, 0xC3, 0x83},
    {0x48, 0x38, 0xA9, 0x2D, 0x19, 0x81, 0x45},
    {0xC1, 0xA1, 0x83, 0x67, 0xA8, 0xAB, 0x81},
    {0x82, 0xDA, 0xB1, 0x84, 0xC8, 0x47, 0x22},
    {0x69, 0x69, 0x39, 0xBC, 0x41, 0x26, 0x3C},
    {0xA4, 0x13, 0x61, 0x29, 0x89, 0x62, 0x56},
    {0x4C, 0x31, 0x23, 0x16, 0x23, 0x4D, 0xD3},
    {0x25, 0x54, 0xB3, 0x4A, 0xD6, 0x65, 0x29},
};

BYTE xor_mask[7] = {0x73,0xC1,0x69,0x08,0x12,0xA6,0x4B};

enum
{
    IDX_SN_MSB  =   0,
    IDX_SN_MID  =   1,
    IDX_SN_LSB  =   2,
    IDX_STATUS  =   3,
    IDX_WC_HIGH =   4,
    IDX_WC_LOW  =   5,
    IDX_CRC     =   6
};


BOOL
decrypt_packet(unsigned char rf_bytes[])
{
    unsigned int i, j;
    BYTE mask, mask1;
    BYTE lsb;
    BYTE crc_bit;
    BYTE data_bit;
    unsigned short payloadCRC;
    BYTE decrypted_buffer[ ENCRYPTED_PACKET_LENGTH ];

    for (i = 0; i < ENCRYPTED_PACKET_LENGTH; i++)
        decrypted_buffer[i] = rf_bytes[i+1];

    //
    // STEP 1    a) look at bits 52,34 & 12 and pick a mask
    //              convert bit52+bit34+bit12 to an index

    mask = (((decrypted_buffer[0] >> 2) & 0x4) + ((decrypted_buffer[2] >> 1) & 0x02) + ((decrypted_buffer[5] >> 4) & 0x01));

    //           b) perform xor on byte0-byte6

    for (i = 0; i < ENCRYPTED_PACKET_LENGTH; i++)
        decrypted_buffer[i] ^= decrypt_mask[mask][i];

    //
    // STEP 2
    //   Modify byte order based on CRC bit 1 and 3

    if ((decrypted_buffer[IDX_CRC] & 0x0a) == 0) {
        // (a) Upper CRC nibble swap with lower S/N msb, (b)lower W/C
        //     count low nibble swap with lower status

        // perform (a)
        mask  = (decrypted_buffer[IDX_CRC] >> 4);
        mask1 = (decrypted_buffer[IDX_SN_MSB] << 4);
        decrypted_buffer[IDX_CRC] &= 0x0f;
        decrypted_buffer[IDX_CRC] |= mask1;
        decrypted_buffer[IDX_SN_MSB] &= 0xf0;
        decrypted_buffer[IDX_SN_MSB] |= mask;
        // perform (b)
        mask  = (decrypted_buffer[IDX_WC_LOW] & 0x0f);
        mask1 = (decrypted_buffer[IDX_STATUS] & 0x0f);
        decrypted_buffer[IDX_WC_LOW] &= 0xf0;
        decrypted_buffer[IDX_WC_LOW] |= mask1;
        decrypted_buffer[IDX_STATUS] &= 0xf0;
        decrypted_buffer[IDX_STATUS] |= mask;
    } else if ((decrypted_buffer[IDX_CRC] & 0x0a) == 0x02) {
        // (a) Upper CRC nibble swap with higher S/N msb, (b) higher W/C
        // count high nibble swap with higher S/N Mid

        //  perform (a)
        mask  = (decrypted_buffer[IDX_CRC] & 0xf0);
        mask1 = (decrypted_buffer[IDX_SN_MSB] & 0xf0);
        decrypted_buffer[IDX_CRC] &= 0x0f;
        decrypted_buffer[IDX_CRC] |= mask1;
        decrypted_buffer[IDX_SN_MSB] &= 0x0f;
        decrypted_buffer[IDX_SN_MSB] |= mask;
        //  perform (b)
        mask  = (decrypted_buffer[IDX_WC_HIGH] & 0xf0);
        mask1 = (decrypted_buffer[IDX_SN_MID] & 0xf0);
        decrypted_buffer[IDX_WC_HIGH] &= 0x0f;
        decrypted_buffer[IDX_WC_HIGH] |= mask1;
        decrypted_buffer[IDX_SN_MID] &= 0x0f;
        decrypted_buffer[IDX_SN_MID] |= mask;

    } else if ((decrypted_buffer[IDX_CRC] & 0x0a) == 0x08) {
        // (a) Upper CRC nibble swap with upper W/C low, (b)lower W/C
        // low nibble swap with upper S/N lsb

        // perform (a)
        mask  = (decrypted_buffer[IDX_CRC] & 0xf0);
        mask1 = (decrypted_buffer[IDX_WC_LOW] & 0xf0);
        decrypted_buffer[IDX_CRC] &= 0x0f;
        decrypted_buffer[IDX_CRC] |= mask1;
        decrypted_buffer[IDX_WC_LOW] &= 0x0f;
        decrypted_buffer[IDX_WC_LOW] |= mask;
        // perform (b)
        mask  = (decrypted_buffer[IDX_WC_LOW] << 4);
        mask1 = (decrypted_buffer[IDX_SN_LSB] >> 4);
        decrypted_buffer[IDX_WC_LOW] &= 0xf0;
        decrypted_buffer[IDX_WC_LOW] |= mask1;
        decrypted_buffer[IDX_SN_LSB] &= 0x0f;
        decrypted_buffer[IDX_SN_LSB] |= mask;

    } else if ((decrypted_buffer[IDX_CRC] & 0x0a) == 0x0a) {
        // (a) Upper CRC nibble swap with lower S/N mid, (b) Upper W/C
        // high nibble swap with lower status

        //  perform (a)
        mask  = (decrypted_buffer[IDX_CRC] >> 4);
        mask1 = (decrypted_buffer[IDX_SN_MID] << 4);
        decrypted_buffer[IDX_CRC] &= 0x0f;
        decrypted_buffer[IDX_CRC] |= mask1;
        decrypted_buffer[IDX_SN_MID] &= 0xf0;
        decrypted_buffer[IDX_SN_MID] |= mask;
        //  perform (b)
        mask  = (decrypted_buffer[IDX_WC_HIGH] >> 4);
        mask1 = (decrypted_buffer[IDX_STATUS] << 4);
        decrypted_buffer[IDX_WC_HIGH] &= 0x0f;
        decrypted_buffer[IDX_WC_HIGH] |= mask1;
        decrypted_buffer[IDX_STATUS] &= 0xf0;
        decrypted_buffer[IDX_STATUS] |= mask;
    }


    //
    // STEP 3
    //   Rotate right CRC times.  CRC does not shift, MSB shifts to Window count low Msbit

    for (i = decrypted_buffer[IDX_CRC]; i > 0; i--) {
        if (decrypted_buffer[IDX_WC_LOW] & 0x01)
            lsb = 0x80;
        else
            lsb = 0x00;
        decrypted_buffer[IDX_WC_LOW] >>=  1;
        if (decrypted_buffer[IDX_WC_HIGH] & 0x01)
            decrypted_buffer[IDX_WC_LOW] |= 0x80;
        decrypted_buffer[IDX_WC_HIGH] >>=  1;
        if (decrypted_buffer[IDX_STATUS] & 0x01)
            decrypted_buffer[IDX_WC_HIGH] |= 0x80;
        decrypted_buffer[IDX_STATUS] >>=  1;
        if (decrypted_buffer[IDX_SN_LSB] & 0x01)
            decrypted_buffer[IDX_STATUS] |= 0x80;
        decrypted_buffer[IDX_SN_LSB] >>=  1;
        if (decrypted_buffer[IDX_SN_MID] & 0x01)
            decrypted_buffer[IDX_SN_LSB] |= 0x80;
        decrypted_buffer[IDX_SN_MID] >>=  1;
        if (decrypted_buffer[IDX_SN_MSB] & 0x01)
            decrypted_buffer[IDX_SN_MID] |= 0x80;
        decrypted_buffer[IDX_SN_MSB] >>=  1;
        decrypted_buffer[IDX_SN_MSB] |= lsb;
    }

    //
    // STEP 4
    //   XOR     with this mask: 0x73,0xC1,0x69,0x08,0x12,0xA6,0x4B
    for (i = 0; i < ENCRYPTED_PACKET_LENGTH; i++)
        decrypted_buffer[i] ^= xor_mask[i];

    //
    // Calculate CRC (12-bit)
    //

    for (i = 0, crcWord = 0; i < 6; i++) {
        for (j = 0; j < 8; j++) {

            // CRC includes only upper nibble of last byte

            if (i == 5 && j == 4)
                break;

            if (crcWord & 0x0800)
                crc_bit = 1;
            else
                crc_bit = 0;

            crcWord <<= 1;

            if (decrypted_buffer[i] & (0x80 >> j))
                data_bit = 1;
            else
                data_bit = 0;

            if (data_bit == 1 && crc_bit == 0)
                crcWord ^= 0x885;
            else if (data_bit == 0 && crc_bit != 0)
                crcWord ^= 0x885;
        }
    }

    // Verify calculated CRC against payload
    payloadCRC =
        ((unsigned short)(decrypted_buffer[IDX_WC_LOW] & 0x0f) << 8) +
        (unsigned short)decrypted_buffer[IDX_CRC];

    crcWord &= 0x0fff;

    if (crcWord == payloadCRC) {

        // Move decrypted bytes into received buffer(s) (don't ask me
        // why there are more than one...)

        for(i = 0; i < ENCRYPTED_PACKET_LENGTH; ++i){
            rcvd_bytes[i + 1] = decrypted_buffer[i];
            rf_bytes[i + 1] = decrypted_buffer[i];
        }

        return true;

    } else

        return false;
}
