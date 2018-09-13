/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef NMEAWIDGET_H
#define NMEAWIDGET_H

#include <QWidget>
#include "tcpbroadcast.h"

namespace Ui {
class NmeaWidget;
}

class NmeaWidget : public QWidget
{
    Q_OBJECT

public:
    explicit NmeaWidget(QWidget *parent = 0);
    ~NmeaWidget();
    void inputNmea(QByteArray msg);
    QString fixType() const;

private slots:
    void on_nmeaLogChooseButton_clicked();
    void on_nmeaLogActiveBox_toggled(bool checked);
    void on_nmeaServerActiveBox_toggled(bool checked);

private:
    Ui::NmeaWidget *ui;
    TcpBroadcast *mNmeaForwardServer;
    QString mFixType;

};

#endif // NMEAWIDGET_H
