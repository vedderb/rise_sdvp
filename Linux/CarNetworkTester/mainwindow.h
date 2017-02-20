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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QUdpSocket>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void udpReadReady();

    void on_sendButton_clicked();
    void on_clearButton_clicked();
    void on_disconnectedButton_toggled(bool checked);
    void on_tcpButton_toggled(bool checked);
    void on_udpButton_toggled(bool checked);
    void on_getStateGenerateButton_clicked();
    void on_addRoutePointGenerateButton_clicked();
    void on_removeLastPointGenerateButton_clicked();
    void on_clearRouteGenerateButton_clicked();
    void on_setAutopilotActiveGenerateButton_clicked();
    void on_getEnuRefGenerateButton_clicked();
    void on_setEnuRefGenerateButton_clicked();
    void on_rcControlGenerateButton_clicked();
    void on_replaceRouteGenerateButton_clicked();
    void on_setStatusPollGenerateButton_clicked();

private:
    Ui::MainWindow *ui;
    QUdpSocket *mUdpSocket;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;

};

#endif // MAINWINDOW_H
