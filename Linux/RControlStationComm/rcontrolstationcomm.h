/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef RCONTROLSTATIONCOMM_H
#define RCONTROLSTATIONCOMM_H

#include "rcontrolstationcomm_global.h"
#include "rcontrolstationcomm_types.h"
#include <QTcpSocket>
#include <QCoreApplication>

class RCONTROLSTATIONCOMMSHARED_EXPORT RControlStationComm
{

public:
    RControlStationComm();
    ~RControlStationComm();
    bool connectTcp(QString host, int port);
    void disconnectTcp();
    void setDebugLevel(int level);
    bool hasError();
    char *lastError();
    void clearBuffers();
    bool getState(int car, CAR_STATE *state, int timeoutMs = 1000);
    bool getEnuRef(int car, bool fromMap, double *llh, int timeoutMs = 1000);
    bool setEnuRef(int car, double *llh, int timeoutMs = 1000);
    bool addRoutePoints(int car, ROUTE_POINT *route, int len,
                        bool replace = false, bool mapOnly = false,
                        int mapRoute = -1, int timeoutMs = 1000);
    bool clearRoute(int car, int mapRoute = -1, int timeoutMs = 1000);
    bool setAutopilotActive(int car, bool active, int timeoutMs = 1000);
    bool rcControl(int car, int mode, double value, double steering);
    bool getRoutePoints(int car, ROUTE_POINT *route, int *len,
                        int maxLen = 500, int mapRoute = -1, int timeoutMs = 1000);
    bool sendTerminalCmd(int car, char *cmd, char *reply, int timeoutMs = 1000);

private:
    struct ERROR_MSG {
        QString command;
        QString description;
    };

    QTcpSocket *mTcpSocket;
    int mDebugLevel;
    QByteArray mRxBuffer;
    QVector<QByteArray> mXmlBuffer;
    QVector<ERROR_MSG> mErrorMsgs;
    char mTextBuffer[10000];

    // CoreApplication
    QCoreApplication *mApp;
    int mAppArgc;
    char const *mAppArgv[2];

    void processData(const QByteArray &data);
    void sendData(const QByteArray &data);
    QByteArray waitForXml(int timeoutMs = 1000);
    bool waitForAck(QString cmd, int timeoutMs = 1000);
    bool isTcpConnected();
    QByteArray requestAnswer(int car, QString cmd, int timeoutMs = 1000);
    bool checkError(QString xml);

};

#endif // RCONTROLSTATIONCOMM_H
