/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BASESTATION_H
#define BASESTATION_H

#include <QWidget>
#include <QTcpSocket>

namespace Ui {
class BaseStation;
}

class BaseStation : public QWidget
{
    Q_OBJECT

public:
    explicit BaseStation(QWidget *parent = 0);
    ~BaseStation();
    int getAvgPosLlh(double &lat, double &lon, double &height);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

    void on_nmeaConnectButton_clicked();
    void on_nmeaSampleClearButton_clicked();


private:
    Ui::BaseStation *ui;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;

    double mXNow;
    double mYNow;
    double mZNow;
    double mXAvg;
    double mYAvg;
    double mZAvg;
    double mAvgSamples;

    QString mFixNowStr;
    QString mSatNowStr;

    void updateNmeaText();
};

#endif // BASESTATION_H
