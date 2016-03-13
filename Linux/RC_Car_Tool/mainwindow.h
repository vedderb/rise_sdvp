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
#include <QList>
#include <QTimer>
#include <QSerialPort>
#include "carinterface.h"
#include "packetinterface.h"

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
    void timerSlot();

    void on_carAddButton_clicked();
    void on_carRemoveButton_clicked();
    void on_serialConnectButton_clicked();
    void on_serialRefreshButton_clicked();
    void on_disconnectButton_clicked();
    void on_mapRemoveTraceButton_clicked();
    void on_MapRemovePixmapsButton_clicked();
    void on_terminalSendButton_clicked();
    void on_terminalClearButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *mTimer;
    QSerialPort *mSerialPort;
    QList<CarInterface*> mCars;
};

#endif // MAINWINDOW_H
