/*
 * EventQ.cpp
 *
 *  Created on: Feb 21, 2015
 *      Author: irtakail
 */

#include <mqueue.h>
#include <syslog.h>
#include <errno.h>
#include <fcntl.h>
#include "EventQ.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define QNAME  "/insynctive_eventq"

static mqd_t s_mq;

void EventQ_Open()
{
    struct mq_attr attr;

    attr.mq_flags = 0;
    attr.mq_maxmsg = 124;
    attr.mq_msgsize = sizeof(struct Event);
    attr.mq_curmsgs = 0;

    mq_unlink(QNAME);           /* Ignore any errors */
    s_mq = mq_open(QNAME, O_RDWR | O_CREAT, 0666, &attr);

    if (s_mq == -1) {
        syslog(LOG_INFO, "mq_open() failed: %d %s\n", errno, strerror(errno));
        exit(-1);
    }
}
void EventQ_Close()
{
    if  (mq_close(s_mq) != -1) {
        syslog(LOG_INFO, "mq_close() failed: %d %s\n", errno, strerror(errno));
    }
}
void EventQ_Send(struct Event *evt)
{
    if (s_mq == -1)
        return;

    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 0;

    if (mq_timedsend(s_mq, (const char *)evt, sizeof(struct Event), 0, &timeout) == -1) {
        if (errno != EINTR)
            syslog(LOG_INFO, "mq_send() failed: %s[%d]\n",
                   strerror(errno), errno);
    }
}


void EventQ_Receive(struct Event *evt)
{
    if (s_mq == -1)
        return;

    if (mq_receive(s_mq, (char *)evt, sizeof(struct Event), 0) < 0)
        if (errno !=  EINTR)
            syslog(LOG_ERR, "mq_receive() failed: %s (%d)", strerror(errno), errno);
}


long EventQ_Depth(struct Event *evt)
{
    if (s_mq == -1)
        return -1;

    struct mq_attr attr;

    if (mq_getattr(s_mq, &attr) < 0) {
        if (errno !=  EINTR)
            syslog(LOG_ERR, "mq_getattr() failed: %s (%d)", strerror(errno), errno);
        return -1;
    }

    return attr.mq_curmsgs;
}
