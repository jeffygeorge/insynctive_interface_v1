/** I N C L U D E S **********************************************************/
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <syslog.h>
#include <semaphore.h>
#include <pthread.h>

#include "EventQ.h"
#include "ESW1032P-01.h"

#define REGPACONFIG      (0x09)
#define POWERLEVEL_5     (0x05)
#define MAX_READ_REG_LEN (10)

struct TxSemaphore
{
    sem_t* semaphore;
    int numMsgSent;
    pthread_mutex_t mutex;
};

extern BYTE txPktQueueCount;
static int s_fd = -1;
static struct TxSemaphore s_TxSem;
static void ResetPic();

/** D E F I N E S ************************************************************/

void radio_data_in(uint8_t address)
{
    static unsigned char buf[MAX_READ_REG_LEN];

    int idx = 0;
    buf[idx++] = SOH;
    buf[idx++] = IMX_SPI_RD_REG;
    buf[idx++] = STX;
    buf[idx++] = address;
    buf[idx++] = ETX;

    if (s_fd != -1)
        write(s_fd, buf, idx);
    return;
}

void radio_powerlevel_set()
{
    static unsigned char buf[MAX_READ_REG_LEN];

    int idx = 0;
    buf[idx++] = SOH;
    buf[idx++] = IMX_SPI_WR_REG;
    buf[idx++] = STX;
    buf[idx++] = REGPACONFIG;
    buf[idx++] = POWERLEVEL_5;
    buf[idx++] = ETX;

    if (s_fd != -1) {
        write(s_fd, buf, idx);
    } else {
        printf("error: write fail :%s, line:%d \n", __func__, __LINE__);
    }

    return;
}

void radio_pic_version_read()
{
    static unsigned char buf[MAX_READ_REG_LEN];

    int idx = 0;
    buf[idx++] = SOH;
    buf[idx++] = IMX_VERSION_REQ;
    buf[idx++] = STX;
    buf[idx++] = ETX;

    if(s_fd != -1)
        write(s_fd, buf, idx);
    return;
}

int radio_read(unsigned char* data_ptr, int maxBytes)
{
    static bool startup = true;

    if (startup) {
        startup = false;
    }

    int numRead = 0;

    if (s_fd != -1)
        numRead = read(s_fd, data_ptr, maxBytes);

    return numRead;
}

static inline bool ValidPacketLength(unsigned char num)
{
    if (num <= ENCRYPTED_PACKET_LENGTH)
        return true;
    else
        return false;
}

#define MAX_RX_LEN    20

int DecodePacket(unsigned char* rxBuf, int numBytes, unsigned char* decodedBuf)
{
    if ((rxBuf == 0) || (decodedBuf == 0))
        return false;

    int i = 0;

    // Look for SOH
    while (rxBuf[i] != SOH) ++i;
    i += 2;

    ++i;
    int j = 0;

    // Look for ETX
    while ((rxBuf[i] != ETX) && (i < numBytes)) {
        if (rxBuf[i] == ESC) ++i;
        decodedBuf[j++] = rxBuf[i++];
    }

    if (rxBuf[i] != ETX)
        return numBytes;

    return j;
}

void* RadioThread(void* __attribute__((unused))unused)
{
#define READ_TIMEOUT  100

    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(s_fd, &rfds);
    unsigned char buf[MAX_RX_LEN];

    int retval;
    struct Event evt;
    int index = 0;

    index = 0;
    bool packet = false;
    enum PACKET_FIELD nextLookaheadChar = SOH;

    unsigned char lastPacket[MAX_RCV_LEN];
    memset(&lastPacket, 0, sizeof(unsigned char) * MAX_RCV_LEN);

    while (1) {
        memset(&evt.data.num, 0, sizeof(unsigned char) * MAX_RCV_LEN);

        retval = radio_read(buf + index, 1);

        if (retval < 0) {

            syslog(LOG_ERR, "RadioThread() Read returned error: %s", strerror(errno));

        } else if (retval > 0) {

            if (buf[index] == nextLookaheadChar) {

                ++index;
                packet = true;
                nextLookaheadChar = STX;

                while (packet && (index < (MAX_RX_LEN - 1))) {
                    retval = radio_read((buf + index), 1);

                    if (retval > 0) {
                        if ((index == 2) && (nextLookaheadChar == STX)) {
                            if (buf[index-2] != SOH) {
                                packet = false;
                            } else {
                                if (buf[index-1] == PIC_SPI_RD_REG_RESP) {
                                    evt.event = REG_EVENT;
                                } else if (buf[index-1] == PIC_RF_RCVD_PKT) {
                                    evt.event = UART_EVENT;
                                } else if (buf[index-1] == PIC_END_OF_TX) {
                                    // do nothing
                                } else if (buf[index-1] == PIC_VERSION_RESP) {
                                    evt.event = RD_VERSION_EVENT;
                                } else {
                                    packet = false;
                                }
                                nextLookaheadChar = ETX;
                            }
                        }

                        if ((buf[index] == nextLookaheadChar) &&
                            (nextLookaheadChar == ETX)) {

                            if (buf[index-1] != ESC) {
                                break;
                            } else {
                                // Previous char was ESC
                                if (buf[index-2] == ESC)
                                    //but the one before it isnt ESC,
                                    //so this is ETX and not a data
                                    //'3'
                                    break;
                            }
                        }

                        ++index;

                    } else {
                        packet = false;
                    }
                }

                if (packet) {
                    if (buf[PIC_RESP_IND] == PIC_END_OF_TX) {
                        HandleEndOfTx();
                    } else {
                        DecodePacket(buf, index+1, evt.data.num);

                        /* if ((evt.event == UART_EVENT) && */
                        /*     ValidPacketLength(evt.data.num[0])) */

                        /*     EventQ_Send(&evt); */

                        /* else // RSSI */

                        if (memcmp(evt.data.num, lastPacket,
                                   sizeof(unsigned char) * MAX_RCV_LEN)) {
                            EventQ_Send(&evt);

                            memcpy(lastPacket, evt.data.num,
                                   sizeof(unsigned char) * MAX_RCV_LEN);
                        }
                    }
                }

                index = 0;
                packet = false;
                nextLookaheadChar = SOH;
            }
        }
    }

    return 0;
}

void OpenUart(int port)
{

#define INSYNC_SERIAL "/dev/ttySP"
#define MAX_PATH_NAME 20

    char path[MAX_PATH_NAME];

    // Initialize UART for Insynctive.
    sprintf(path, INSYNC_SERIAL"%d", port);
    printf("Open port: %s\n", path);

    s_fd = open(path, O_RDWR | O_NOCTTY);

    if (s_fd < 0) {
        syslog(LOG_INFO, "OpenUart() error %d opening %s: %s",
               errno, path, strerror (errno));
        exit(-1);
    }
}

void SetupUart()
{
    struct termios oldtio,newtio;
    tcgetattr(s_fd, &oldtio); /* save current serial port settings */
    newtio = oldtio;
    newtio.c_lflag = 0;

    /* initialize all control characters*/
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
      now clean the modem line and activate the settings for the port
    */
    tcsetattr(s_fd, TCSANOW, &newtio);
}

/*
 *  radio_init
 *
 *  Initialize radio to correct frequency, data rate, etc.
 *
 *  in:
 *
 *  out:
 *
 *
 */
void radio_init(int port)
{
    OpenUart(port);
    SetupUart();
    radio_powerlevel_set();

    // Initialize Tx/Ack queue count
    txPktQueueCount = 0;

    s_TxSem.semaphore = sem_open("/TxSemaphore", O_CREAT, 0777, 1);

    if (SEM_FAILED == s_TxSem.semaphore) {
        printf("Tx Semaphore sem_open failure %d\n", errno);
        exit(EXIT_FAILURE);
    }

    pthread_mutexattr_t attr;

    int ret = pthread_mutexattr_init(&attr);

    if (ret != 0) {
        printf("pthread_mutexattr_init failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);

    if (ret != 0) {
        printf("pthread_mutexattr_init failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }


    ret = pthread_mutex_init(&s_TxSem.mutex, &attr);

    if (ret != 0) {
        printf("pthread_mutex_init failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }
}

static inline bool EscChar(unsigned char a)
{
    if(a == SOH || a == ETX || a == STX || a == ESC)
        return true;
    else
        return false;
}

void radio_transmit_packet(unsigned char rf_bytes[], int num)
{

#define MAX_PKT_LEN 20
#define CMD_IND 1
#define HEADER_LEN 3

    unsigned char pkt[MAX_PKT_LEN] = {SOH, IMX_INSYNC_TX, STX};

    if ((rf_bytes[1] >> 3) == DEVICE_TYPE_WINDOW_SHADE) {
        pkt[CMD_IND] = IMX_INSYNC_SHADE_TX;
    }

    int j = HEADER_LEN;
    for (int i = 0; i < num; ++i, ++j) {
        if (EscChar(rf_bytes[i]))
            pkt[j++] = ESC;

        pkt[j] = rf_bytes[i];
    }

    pkt[j++] = ETX;
    pthread_mutex_lock(&s_TxSem.mutex);

    if (s_TxSem.numMsgSent == 0) {

#ifdef SEM
        sem_wait(s_TxSem.semaphore);
#endif

    }

    write(s_fd, &pkt, j);
    ++s_TxSem.numMsgSent;

    if (s_TxSem.numMsgSent > 5) {
        int fd = s_fd;
        s_fd = -1;
        close(fd);
        ResetPic();
        OpenUart(0);
        s_TxSem.numMsgSent = 1;
    }

    pthread_mutex_unlock(&s_TxSem.mutex);
}

void HandleEndOfTx()
{
    pthread_mutex_lock(&s_TxSem.mutex);

    if (s_TxSem.numMsgSent == 1) {

#ifdef SEM
        sem_post(s_TxSem.semaphore);
#endif

    }

    if (s_TxSem.numMsgSent)
        --s_TxSem.numMsgSent;

    pthread_mutex_unlock(&s_TxSem.mutex);
}

void ResetPic()
{
    system("/bridge/manufacturing/gpio.sh 44 out 1");
    usleep(1000);
    system("/bridge/manufacturing/gpio.sh 44 out 0");
}
