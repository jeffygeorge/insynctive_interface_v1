#include "errno.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <syslog.h>
#include <string.h>

#include "Timer.h"

static void Handler(int sig, siginfo_t *si, void *uc)
{
    // Send event back to main
    struct Event msg;

    if (si)
        msg.event = (enum EventType)si->si_value.sival_int;
    else
        msg.event = TIMER_EVENT;

    EventQ_Send(&msg);
}

void TimerStart(timer_t timerId, int sec, int msec, int it_sec, int it_msec)
{
    struct itimerspec its;

    /* Start the timer */

    its.it_value.tv_sec = sec;
    its.it_value.tv_nsec = MILLI_TO_NANOSEC(msec);
    its.it_interval.tv_sec = it_sec;
    its.it_interval.tv_nsec = MILLI_TO_NANOSEC(it_msec);

    if (timer_settime(timerId, 0, &its, NULL) == -1) {
        syslog(LOG_ERR, "TimerStart() error: %d / %s", errno, strerror(errno));
        exit(-1);
    }
}

void TimerStop(timer_t timerId)
{
    struct itimerspec its = {{0,0}, {0,0}};

    if (timer_settime(timerId, 0, &its, NULL) == -1) {
        syslog(LOG_ERR, "TimerStop() timer_settime failed: %d / %s",
               errno, strerror(errno));
        exit(-1);
    }
}

void TimerDisable(timer_t timerId)
{
    if (TimerIsActive(timerId))
    {
        struct itimerspec timerspec = { {0,0}, {0,0} };

        if (timer_settime(timerId, 0, &timerspec, 0) == -1) {
            syslog(LOG_ERR, "TimerDisable() timer_settime failed: %d / %s",
                   errno, strerror(errno));
        }
    }
}

void TimerInit(timer_t *timerId, enum EventType evt)
{
    struct sigevent sev;
    struct sigaction sa;

    /* Establish handler for timer signal */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = Handler;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGRTMIN, &sa, NULL) < 0) {
        syslog(LOG_ERR, "sigaction() failed: %s", strerror(errno));
        exit(-1);
    }

    /* Create the timer */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_int = evt;

    if (timer_create(CLOCK_MONOTONIC, &sev, timerId) < 0) {
        syslog(LOG_ERR, "Unable to create timer: %s", strerror(errno));
        exit(-1);
    }
}


bool TimerIsActive(timer_t timerId)
{
    struct itimerspec timerspec;

    if (timer_gettime(timerId, &timerspec) == -1)
        syslog(LOG_ERR, "TimerIsActive() timer_gettime failed: %d %s",
               errno, strerror(errno));

    return ((timerspec.it_value.tv_sec != 0) || (timerspec.it_value.tv_nsec != 0));
}
