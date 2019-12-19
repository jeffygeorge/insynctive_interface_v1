/*
 *  Created on: Feb 20, 2015
 *      Author: irtakail
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>

#include <sys/inotify.h>
#include <sys/select.h>
#include <sys/syscall.h>

#include <sqlite3.h>

#include "EventQ.h"
#include "shade_transmit_queue.h"
#include "ESW1032P-01.h"
#include "DbInterface.h"


const uint8_t SQL_STMT_LEN = 100;
const uint32_t MILLISECOND = 1000000;
const char *SQLITE3_DB = "/bridge/db/bridge.sqlite3";

extern struct NODE_ID_TO_SN nodeIDtoSN[MAX_DEVICES];

int inotifyFd;

struct DeviceStatus
{
    enum DEVICE_TYPE type;
    enum ATTRIBUTE attrib;
};

static struct DeviceStatus s_DevStatus[] =
{
    {DEVICE_TYPE_DOOR_WINDOW_SENSOR, BATT_LEVEL},
    {DEVICE_TYPE_DOOR_WINDOW_SENSOR, IS_OPEN},
    {DEVICE_TYPE_DOOR_WINDOW_SENSOR, IS_TAMPERED},
    {DEVICE_TYPE_DOOR_WINDOW_SENSOR, IS_SUPER_FAULT},

    {DEVICE_TYPE_TILT_SENSOR, BATT_LEVEL},
    {DEVICE_TYPE_TILT_SENSOR, IS_OPEN},
    {DEVICE_TYPE_TILT_SENSOR, IS_TAMPERED},
    {DEVICE_TYPE_TILT_SENSOR, IS_SUPER_FAULT},

    {DEVICE_TYPE_DEADBOLT_SENSOR, BATT_LEVEL},
    {DEVICE_TYPE_DEADBOLT_SENSOR, IS_LOCKED},
    {DEVICE_TYPE_DEADBOLT_SENSOR, IS_TAMPERED},
    {DEVICE_TYPE_DEADBOLT_SENSOR, IS_SUPER_FAULT},

    {DEVICE_TYPE_INSYNCTIVE_BLIND_REMOTE, BATT_LEVEL},
    {DEVICE_TYPE_INSYNCTIVE_BLIND_REMOTE, IS_SUPER_FAULT},

    {DEVICE_TYPE_WINDOW_SHADE, BATT_LEVEL},
    {DEVICE_TYPE_WINDOW_SHADE, POSITION},
    {DEVICE_TYPE_WINDOW_SHADE, ANGLE},
    {DEVICE_TYPE_WINDOW_SHADE, LOWER_SOFT_LIMIT},
    {DEVICE_TYPE_WINDOW_SHADE, UPPER_SOFT_LIMIT},
    {DEVICE_TYPE_WINDOW_SHADE, SAVED_POSITION},
    {DEVICE_TYPE_WINDOW_SHADE, IS_SUPER_FAULT},
    {DEVICE_TYPE_WINDOW_SHADE, SHADE_ACK},

    {DEVICE_TYPE_STATUS_MODULE, BATT_LEVEL},
    {DEVICE_TYPE_STATUS_MODULE, IS_SUPER_FAULT}

    /* {DEVICE_TYPE_BOOM_BRIDGE, BATT_LEVEL} // Bridge - Battery Level */
};

void DbOpen(sqlite3 **db, pthread_mutex_t *mutex, const char *func, int line)
{
    /* If we weren't given a valid pointer, just return */
    if (!db) return;

    /* If we were given an existing database connection, this is a
       no-op; just return */
    if (*db) return;

    int dbOpenResult = sqlite3_open(SQLITE3_DB, db);

    if (dbOpenResult != SQLITE_OK) {
        syslog(LOG_INFO, "Cannot open %s: %s - aborting",
               SQLITE3_DB, sqlite3_errmsg(*db));
        *db = NULL;
        return;
    }

    sqlite3_busy_timeout(*db, 2000);
}

int DbClose(sqlite3 *db, pthread_mutex_t *mutex, const char *func, int line)
{
    if (!db) return SQLITE_OK;

    int retval = sqlite3_close(db);

    return retval;
}

void handleDbError(const char *func, int line, int response) {
    syslog(LOG_ERR, "%s[%04d] Unexpected database response: %s[%d]",
           func, line, sqlite3_errstr(response), response);
}

#define SENSOR_TRUE "true"
#define SENSOR_FALSE "false"

inline bool IsAttributeBool(enum ATTRIBUTE attrib)
{
    switch(attrib)
    {
    case IS_OPEN:
    case IS_LOCKED:
    case IS_TAMPERED:
    case IS_SUPER_FAULT:
    case SHADE_ACK:
        return true;
    default:
        return false;
    }
}


unsigned char DbGetTotalDevices(sqlite3 *passedDb, pthread_mutex_t *mutex)
{
    int totalDevices = 0;
    sqlite3_stmt *res;
    int rc;
    int status;

    const char *sql = "SELECT COUNT(*) FROM insynctive_paired_devices";

    sqlite3 *db = passedDb;

    DbOpen(&db, mutex, __func__, __LINE__);

    rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc != SQLITE_OK) {
        handleDbError(__func__, __LINE__, rc);
    } else {
        status = sqlite3_step(res);

        if (status == SQLITE_ROW) {
            totalDevices = sqlite3_column_int(res, 0);
        } else {
            handleDbError(__func__, __LINE__, status);
        }
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);

    return totalDevices;
}

unsigned long DBReadBoomSerialNum(sqlite3 *passedDb, pthread_mutex_t *mutex)
{
    sqlite3_stmt *res;
    unsigned long retval = 0;
    int status;

    const char *sql =
        "SELECT serial_number FROM insynctive_paired_devices WHERE \"type#\"=?";

    sqlite3 *db = passedDb;

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, sql, strlen(sql), &res, NULL);

    if (rc == SQLITE_OK) {

        rc = sqlite3_bind_int(res, 1, DEVICE_TYPE_BOOM_BRIDGE);

        if (rc == SQLITE_OK) {
            status = sqlite3_step(res);

            if (status == SQLITE_ROW) {
                const unsigned char *textResult = sqlite3_column_text(res, 0);
                retval = strtol((const char *)textResult, NULL, 0);
            } else {
                handleDbError(__func__, __LINE__, status);
            }

        } else {
            handleDbError(__func__, __LINE__, rc);
        }

    } else {
        handleDbError(__func__, __LINE__, rc);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);

    return retval;
}


void DbReadPairedDevices(sqlite3 *passedDb, pthread_mutex_t *mutex)
{
    sqlite3_stmt *res;
    struct NODE_ID_TO_SN n;
    enum PairedDevicesInd {
        ID,
        TYPE,
        SN
    };

    const char *query = "SELECT * FROM insynctive_paired_devices";

    sqlite3 *db = passedDb;

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, query, -1, &res, 0);

    if (rc != SQLITE_OK) {
        handleDbError(__func__, __LINE__, rc);
        sqlite3_finalize(res);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    do {
        rc = sqlite3_step(res);

        if (rc == SQLITE_ROW) {
            memset(&n, 0, sizeof(struct NODE_ID_TO_SN));
            n.id = sqlite3_column_int(res, ID);
            n.device_type = (enum DEVICE_TYPE)sqlite3_column_int(res, TYPE);
            n.SN = strtol((const char*)sqlite3_column_text(res, SN), NULL, 0);

            if (n.device_type == DEVICE_TYPE_STATUS_MODULE)
                AddStatusModule(n.id, n.SN);
            else if (n.device_type != DEVICE_TYPE_BOOM_BRIDGE)
                AddToNodeIDTable(&n);

        } else if (rc != SQLITE_DONE) {
            handleDbError(__func__, __LINE__, rc);
        }
    } while (rc == SQLITE_ROW);

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}


void GetId(sqlite3 *passedDb, pthread_mutex_t *mutex, unsigned long sn,
           unsigned char *id)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    const char *sql =
        "select \"id#\" from insynctive_paired_devices where serial_number=?";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc == SQLITE_OK) {

        char *buffer;
        asprintf(&buffer, "0x%lx", sn);

        rc = sqlite3_bind_text(res, 1, buffer, -1, free);

        if (rc == SQLITE_OK) {

            rc = sqlite3_step(res);

            if ((rc == SQLITE_ROW) || (rc == SQLITE_DONE)) {
                *id = sqlite3_column_int(res, 0);
            } else {
                handleDbError(__func__, __LINE__, rc);
            }
        } else {
            handleDbError(__func__, __LINE__, rc);
        }
    } else {
        handleDbError(__func__, __LINE__, rc);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}


void AddDefaultStatus(sqlite3 *db, pthread_mutex_t *mutex, unsigned char id,
                      enum DEVICE_TYPE type, bool newInsertion)
{
    clock_t now = clock();

    for (int i = 0; i < (int)(sizeof(s_DevStatus) / sizeof(s_DevStatus[0])); ++i) {
        if (s_DevStatus[i].type == type) {

            if (IsAttributeBool(s_DevStatus[i].attrib))

                if ((s_DevStatus[i].attrib == IS_OPEN) ||
                    (s_DevStatus[i].attrib == IS_LOCKED))

                    UpdateStatus(db, mutex, id, type, s_DevStatus[i].attrib,
                                 true, &now, newInsertion);

                else

                    UpdateStatus(db, mutex, id, type, s_DevStatus[i].attrib,
                                 false, &now, newInsertion);

            else

                if (s_DevStatus[i].attrib == BATT_LEVEL &&
                    (type == DEVICE_TYPE_TILT_SENSOR || type == DEVICE_TYPE_DOOR_WINDOW_SENSOR ||
                     type == DEVICE_TYPE_DEADBOLT_SENSOR))

                    UpdateStatus(db, mutex, id, type, s_DevStatus[i].attrib,
                                 100, &now, newInsertion);

                else

                    UpdateStatus(db, mutex, id, type, s_DevStatus[i].attrib,
                                 0, &now, newInsertion);

            sleep(1);
        }
    }
}

void AddPairedDevice(sqlite3 *passedDb, pthread_mutex_t *mutex, unsigned long sn,
                     enum DEVICE_TYPE type, unsigned char* id)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    const char *sql =
        "INSERT INTO insynctive_paired_devices (\"type#\", serial_number) VALUES (?, ?)";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc == SQLITE_OK)
        while (1) {

            rc = sqlite3_bind_int(res, 1, type);

            if (rc != SQLITE_OK)
                break;

            char *buffer;
            asprintf(&buffer, "0x%lx", sn);
            rc = sqlite3_bind_text(res, 2, buffer, -1, free);

            if (rc != SQLITE_OK)
                break;

            rc = sqlite3_step(res);

            if (rc != SQLITE_DONE)
                syslog(LOG_ERR, "AddPairedDevice() Unexpected response: %s", sqlite3_errstr(rc));

            if (rc != SQLITE_BUSY) {
                handleDbError(__func__, __LINE__, rc);
                break;
            }
        }

    if (rc == SQLITE_DONE) {
        GetId(db, mutex, sn, id);
        AddDefaultStatus(db, mutex, *id, type, true);
        DbAddToBLCmdTable(db, mutex, BL_UPDATE_BONJOUR, 0);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}

static void RemoveDeviceFromStatusTbl(sqlite3 *passedDb, pthread_mutex_t *mutex,
                                      unsigned char id)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    const char *sql =
        "DELETE FROM insynctive_device_status WHERE \"id#\" = ?";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc == SQLITE_OK) {

        rc = sqlite3_bind_int(res, 1, id);

        if (rc == SQLITE_OK) {

            rc = sqlite3_step(res);

            if (rc != SQLITE_DONE)
                handleDbError(__func__, __LINE__, rc);
        } else {
            handleDbError(__func__, __LINE__, rc);
        }
    } else {
        handleDbError(__func__, __LINE__, rc);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}

void RemovePairedDevice(sqlite3 *passedDb, pthread_mutex_t *mutex,
                        unsigned char id)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    const char *sql =
        "DELETE FROM insynctive_paired_devices WHERE \"id#\" = ?";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc == SQLITE_OK) {

        rc = sqlite3_bind_int(res, 1, id);

        if (rc == SQLITE_OK) {

            rc = sqlite3_step(res);

            if (rc != SQLITE_DONE)
                handleDbError(__func__, __LINE__, rc);
        } else {
            handleDbError(__func__, __LINE__, rc);
        }
    }

    if (rc == SQLITE_OK) {
        handleDbError(__func__, __LINE__, rc);
        RemoveDeviceFromStatusTbl(db, mutex, id);
        DbAddToBLCmdTable(db, mutex, BL_UPDATE_BONJOUR, 0);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}


bool CheckDeviceStatusAttribute(sqlite3 *passedDb, pthread_mutex_t *mutex,
                                unsigned char id, enum ATTRIBUTE attrib)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;
    bool status = true;

    const char *sql =
        "SELECT EXISTS("
        "  SELECT 1 FROM insynctive_device_status"
        "   WHERE \"id#\" = ?"
        "     AND \"attribute#\" = ?);";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare(db, sql, strlen(sql), &res, 0);

    if (rc == SQLITE_OK) {
        rc = sqlite3_step(res);
    } else {
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return status;
    }

    if (rc == SQLITE_OK)
        rc = sqlite3_bind_int(res, 1, id);
    else
        handleDbError(__func__, __LINE__, rc);

    if (rc == SQLITE_OK)
        rc = sqlite3_bind_int(res, 2, attrib);
    else
        handleDbError(__func__, __LINE__, rc);

    if (rc == SQLITE_OK) {
        if ((rc == SQLITE_ROW) || (rc == SQLITE_DONE)) {
            if (sqlite3_column_int(res, 0) == 0) {
                status = false;
            }
        } else {
            handleDbError(__func__, __LINE__, rc);
        }
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);

    return status;
}

void UpdateStartupValues(sqlite3 *passedDb, pthread_mutex_t *mutex)
{
#define SQL_UPDATE_STMT_LEN 150
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    /* Set all sensors to "open" */

    const char *query =
        "UPDATE insynctive_device_status"
        "   SET value='true', timestamp = DATETIME('now','localtime')"
        " WHERE \"attribute#\"=2";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, query, -1, &res, 0);

    if (rc == SQLITE_OK) {

        rc = sqlite3_step(res);

        if (rc == SQLITE_DONE)
            sqlite3_reset(res);
        else
            handleDbError(__func__, __LINE__, rc);

    } else {
        handleDbError(__func__, __LINE__, rc);
    }

    rc = sqlite3_finalize(res);

    /* Set all locks to unlocked */

    query =
        "UPDATE insynctive_device_status"
        "   SET value='false', timestamp = DATETIME('now','localtime')"
        " WHERE \"attribute#\"=3";

    rc = sqlite3_prepare_v2(db, query, -1, &res, 0);

    if (rc == SQLITE_OK) {
        rc = sqlite3_step(res);

        if (rc == SQLITE_DONE)
            sqlite3_reset(res);
        else
            handleDbError(__func__, __LINE__, rc);

    } else {
        handleDbError(__func__, __LINE__, rc);
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}


void UpdateStatus(sqlite3 *passedDb, pthread_mutex_t *mutex, unsigned char id,
                  enum DEVICE_TYPE type, enum ATTRIBUTE attrib,
                  unsigned char currStatus, const clock_t *timestamp,
                  bool addDefault)
{
#define SQL_UPDATE_STMT_LEN 150
    const char *sql = NULL;
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    if (addDefault == false)
        sql =
            "UPDATE insynctive_device_status"
            "   SET value=?, timestamp = datetime(\'now\',\'localtime\')"
            " WHERE \"id#\" = ?"
            "   AND \"attribute#\" = ?";
    else
        sql =
            "INSERT INTO insynctive_device_status (value, \"id#\", \"attribute#\", timestamp)"
            "  VALUES (?, ?, ?, datetime(\"now\", \"localtime\"))";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, sql, -1, &res, 0);

    if (rc == SQLITE_OK) {
        if (IsAttributeBool(attrib)) {
            if (currStatus == true) {
                rc = sqlite3_bind_text(res, 1, "true", 4, NULL);
            } else {
                rc = sqlite3_bind_text(res, 1, "false", 5, NULL);
            }
        } else {
            rc = sqlite3_bind_int(res, 1, currStatus);
        }

        if (rc == SQLITE_OK)
            rc = sqlite3_bind_int(res, 2, id);
        else
            handleDbError("UpdateStatus/bind", __LINE__, rc);

        if (rc == SQLITE_OK)
            rc = sqlite3_bind_int(res, 3, attrib);
        else
            handleDbError("UpdateStatus/bind", __LINE__, rc);

        if (rc == SQLITE_OK) {
            rc = sqlite3_step(res);

            if ((rc != SQLITE_DONE) && (rc != SQLITE_OK))
                handleDbError("UpdateStatus/step", __LINE__, rc);
        }

    } else {
        handleDbError("UpdateStatus/prepare", __LINE__, rc);
    }

    rc = sqlite3_finalize(res);

    if (rc != SQLITE_OK)
        handleDbError("UpdateStatus/finalize", __LINE__, rc);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}

void DbAddToBLCmdTable(sqlite3 *passedDb, pthread_mutex_t *mutex,
                       enum BLCommands blCmd, char *value)
{
    if (!value) {
        syslog(LOG_ERR, "DbAddToBLCmdTable(): NULL value received");
        return;
    }

    sqlite3_stmt *statement;
    sqlite3 *db = passedDb;

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db,
                                "INSERT INTO business_logic_commands(`blCommand#`, `blAttribute#`, value)"
                                "VALUES (?, ?, ?)",
                                -1, &statement, 0);

    if (rc != SQLITE_OK) {
        sqlite3_finalize(statement);
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    /* At this point, we know that the statement is ready for parameters */

    rc = sqlite3_bind_int(statement, 1, blCmd);

    if (rc != SQLITE_OK) {
        sqlite3_finalize(statement);
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    rc = sqlite3_bind_text(statement, 3, value, -1, free);

    if (rc != SQLITE_OK) {
        sqlite3_finalize(statement);
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    /* If this is a tampered or untampered message, the attribute #
     * should be "1", otherwise it should be left as NULL */

    if ((blCmd == BL_INDICATE_UNTAMPERED) || (blCmd == BL_INDICATE_TAMPERED)) {
        rc = sqlite3_bind_int(statement, 2, 1);
    } else {
        rc = sqlite3_bind_null(statement, 2);
    }

    if (rc != SQLITE_OK) {
        sqlite3_finalize(statement);
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    rc = sqlite3_step(statement);

    if (rc != SQLITE_DONE) {
        handleDbError(__func__, __LINE__, rc);
    }

    sqlite3_finalize(statement);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}

void DbClearCmdTbl(sqlite3 *passedDb, pthread_mutex_t *mutex, int i)
{
    sqlite3_stmt *res;
    sqlite3 *db = passedDb;

    const char *query = "DELETE FROM insynctive_commands WHERE primaryKey=?";

    DbOpen(&db, mutex, __func__, __LINE__);

    int rc = sqlite3_prepare_v2(db, query, -1, &res, 0);

    if (rc != SQLITE_OK) {
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    rc = sqlite3_bind_int(res, 1, i);
    if (rc != SQLITE_OK) {
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    rc = sqlite3_step(res);
    if ((rc != SQLITE_OK) && (rc != SQLITE_DONE)) {
        handleDbError(__func__, __LINE__, rc);
        if (!passedDb)
            DbClose(db, mutex, __func__, __LINE__);
        return;
    }

    sqlite3_finalize(res);

    if (!passedDb)
        DbClose(db, mutex, __func__, __LINE__);
}

void DBMonitorCommandTable(pthread_mutex_t *mutex)
{
    sqlite3_stmt *res;
    sqlite3 *db = NULL;
    struct Event evt;
    enum CmdTblInd {
        KEY,
        COMMAND,
        ID,
        ATTR,
        VALUE
    };

    const char *query =
        "SELECT primaryKey, \"iCommand#\", \"id#\", \"attribute#\", value "
        "  FROM insynctive_commands";

    while (true) {

        DbOpen(&db, mutex, __func__, __LINE__);

        int rc = sqlite3_prepare_v2(db, query, -1, &res, 0);


        if (rc != SQLITE_OK) {
            sqlite3_finalize(res);
            handleDbError(__func__, __LINE__, rc);
            DbClose(db, mutex, __func__, __LINE__);
            return;
        }

        rc = sqlite3_step(res);

        if (rc == SQLITE_BUSY) {
            sqlite3_finalize(res);
            handleDbError(__func__, __LINE__, rc);
            DbClose(db, mutex, __func__, __LINE__);
            return;
        }

        if (rc == SQLITE_DONE) {
            sqlite3_finalize(res);
            DbClose(db, mutex, __func__, __LINE__);
            break;
        }

        if (rc != SQLITE_ROW) {
            handleDbError(__func__, __LINE__, rc);
            sqlite3_finalize(res);
            DbClose(db, mutex, __func__, __LINE__);
            break;
        }

        evt.event = DB_EVENT;
        evt.data.dbEvent.key = sqlite3_column_int(res, KEY);
        evt.data.dbEvent.cmd = (enum DbCommand)sqlite3_column_int(res, COMMAND);
        evt.data.dbEvent.id = sqlite3_column_int(res, ID);
        evt.data.dbEvent.attrib = (enum ATTRIBUTE)sqlite3_column_int(res, ATTR);

        if (evt.data.dbEvent.attrib == TARGET_POSITION)
            evt.data.dbEvent.attrib = POSITION;
        else if (evt.data.dbEvent.attrib == TARGET_ANGLE)
            evt.data.dbEvent.attrib = ANGLE;

        if (sqlite3_column_text(res, VALUE)) {
            if (!IsAttributeBool(evt.data.dbEvent.attrib)) {
                strncpy(evt.data.dbEvent.value,
                        (const char *)sqlite3_column_text(res, VALUE),
                        MAX_RCV_LEN);
            } else {
                if (strncmp("true", (const char*)sqlite3_column_text(res, VALUE), 4)) {
                    strcpy(evt.data.dbEvent.value, "1");
                } else {
                    strcpy(evt.data.dbEvent.value, "0");
                }
            }
        } else {
            memset(evt.data.dbEvent.value, 0, MAX_RCV_LEN);
        }

        sqlite3_finalize(res);

        DbClose(db, mutex, __func__, __LINE__);

        EventQ_Send(&evt);

        DbClearCmdTbl(NULL, mutex, evt.data.dbEvent.key);
    }
}

void* DBThread(void* arg)
{
    int result;
    fd_set readFds;
    struct timeval interval;
    pthread_mutex_t *mutex = (pthread_mutex_t *)arg;
    struct inotify_event event;
    int inotifyFd;

    inotifyFd = inotify_init(); /* Create inotify instance */

    if (inotifyFd == -1) {
        syslog(LOG_INFO, "ionotifyfd create failed %d %s\n", errno, strerror(errno));
        exit(-1);
    }

    inotify_add_watch(inotifyFd, SQLITE3_DB, IN_MODIFY);

    while(1) {

        FD_ZERO(&readFds);
        FD_SET(inotifyFd, &readFds);

        int maxFd = inotifyFd;

        interval.tv_sec = 1;
        interval.tv_usec = 0;

        result = select(maxFd + 1, &readFds, NULL, NULL, &interval);

        if (result > 0) {
            read(inotifyFd, &event, sizeof(event));

            if (!(event.mask & IN_MODIFY))
                continue;
        }

        DBMonitorCommandTable(mutex);
    }

    return NULL;
}
