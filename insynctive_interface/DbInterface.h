/*
 * DbInterface.h
 *
 *  Created on: Feb 18, 2015
 *      Author: irtakail
 */

#ifndef DBINTERFACE_H_
#define DBINTERFACE_H_

#include <pthread.h>
#include <time.h>

#include "sqlite3.h"

#include "ESW1032P-01.h"

enum BLCommands
{
    BL_LEARN_SUCCESS = 3,
    BL_UPDATE_BONJOUR = 17,
    BL_UNLEARN_SUCCESS = 18,
    BL_SENSOR_TEST_SUCCEEDED = 19,
    BL_SENSOR_TEST_FAILED = 20,
    BL_INDICATE_TAMPERED = 28,
    BL_INDICATE_UNTAMPERED = 30,
    BL_LEARN_REQUEST = 31,
    BL_UNLEARN_REQUEST = 32
};

void DbOpen(sqlite3**, pthread_mutex_t *, const char *, int);
void DBClose(sqlite3*, pthread_mutex_t *, const char *, int);

unsigned char DbGetTotalDevices(sqlite3 *, pthread_mutex_t *);
unsigned long DBReadBoomSerialNum(sqlite3 *, pthread_mutex_t *);
void GetId(sqlite3 *, pthread_mutex_t *, unsigned long, unsigned char *);
void AddDefaultStatus(sqlite3 *, pthread_mutex_t *, unsigned char,
                      enum DEVICE_TYPE, bool);
void AddPairedDevice(sqlite3 *, pthread_mutex_t *, unsigned long,
                     enum DEVICE_TYPE, unsigned char *);
void RemovePairedDevice(sqlite3 *, pthread_mutex_t *, unsigned char);
bool CheckDeviceStatusAttribute(sqlite3 *, pthread_mutex_t *,
                                unsigned char, enum ATTRIBUTE);
void UpdateStatus(sqlite3*, pthread_mutex_t*, unsigned char,
                  enum DEVICE_TYPE, enum ATTRIBUTE, unsigned char,
                  const clock_t *, bool);
void DbAddToBLCmdTable(sqlite3*, pthread_mutex_t *, enum BLCommands, char *);
void DbClearCmdTbl(sqlite3*, pthread_mutex_t *, int);

void DbReadPairedDevices(sqlite3*, pthread_mutex_t *);
void UpdateStartupValues(sqlite3*, pthread_mutex_t *);

void* DBThread(void*);

#endif
