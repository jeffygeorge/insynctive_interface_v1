/*
 * EventQ.h
 *
 *  Created on: Feb 21, 2015
 *      Author: irtakail
 */

#ifndef EVENTQ_H_
#define EVENTQ_H_

#include <time.h>
#include "ESW1032P-01.h"

enum EventType
{
    DB_EVENT = 1,
    UART_EVENT,             //2
    REG_EVENT,              //3
    TIMER_EVENT,            //4
    SM_EVENT,               //5
    SHADE_DELAY_EVENT,      //6
    SHADE_TX_EVENT,         //7
    STATUS_MODULE_TX_EVENT,  //8
    RD_VERSION_EVENT
};

enum ATTRIBUTE;

#define MAX_RCV_LEN 64

struct DbEvent
{
    unsigned char key;
    enum DbCommand cmd;
    unsigned char id;
    enum ATTRIBUTE attrib;
    char value[MAX_RCV_LEN];
};

struct Event
{
    enum EventType event;
    union
    {
        unsigned char num[MAX_RCV_LEN];
        struct DbEvent dbEvent;
    } data;
};

void EventQ_Open();
void EventQ_Close();
void EventQ_Send(struct Event *evt);
void EventQ_Receive(struct Event *evt);
long EventQ_Depth(struct Event *evt);

#endif /* EVENTQ_H_ */
