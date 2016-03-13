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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    on_serialRefreshButton_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timerSlot()
{

}

void MainWindow::on_carAddButton_clicked()
{
    CarInterface *car = new CarInterface(this);
    int id = mCars.size();
    mCars.append(car);
    QString name;
    name.sprintf("Car %d", id);
    car->setID(id);
    ui->carsWidget->addTab(car, name);
}

void MainWindow::on_carRemoveButton_clicked()
{
    if (mCars.size() > 0) {
        CarInterface *car = (CarInterface*)ui->carsWidget->currentWidget();
        ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
        mCars.removeOne(car);
        delete car;
    }
}

void MainWindow::on_serialConnectButton_clicked()
{

}

void MainWindow::on_serialRefreshButton_clicked()
{
    ui->serialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        QString name = port.portName();
        int index = ui->serialPortBox->count();
        // put STMicroelectronics device first in list and add prefix
        if(port.manufacturer() == "STMicroelectronics") {
            name.insert(0, "IF - ");
            index = 0;
        }
        ui->serialPortBox->insertItem(index, name, port.systemLocation());
    }

    ui->serialPortBox->setCurrentIndex(0);
}

void MainWindow::on_disconnectButton_clicked()
{

}

void MainWindow::on_mapRemoveTraceButton_clicked()
{

}

void MainWindow::on_MapRemovePixmapsButton_clicked()
{

}

void MainWindow::on_terminalSendButton_clicked()
{

}

void MainWindow::on_terminalClearButton_clicked()
{

}
