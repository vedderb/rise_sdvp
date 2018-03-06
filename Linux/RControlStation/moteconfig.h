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

#ifndef MOTECONFIG_H
#define MOTECONFIG_H

#include <QWidget>
#include "datatypes.h"
#include "packetinterface.h"

namespace Ui {
class MoteConfig;
}

class MoteConfig : public QWidget
{
    Q_OBJECT

public:
    explicit MoteConfig(QWidget *parent = 0);
    ~MoteConfig();
    void setPacketInterface(PacketInterface *packetInterface);

private slots:
    void terminalPrint(quint8 id, QString str);

    void on_setConfigButton_clicked();
    void on_terminalSendButton_clicked();
    void on_ubxPollNavSvinButton_clicked();
    void on_ubxPollNavRelPosNedButton_clicked();
    void on_ubxPollRxmRawxButton_clicked();
    void on_ubxPollNavSolButton_clicked();
    void on_m8tBaseStatusButton_clicked();
    void on_m8tBaseResetButton_clicked();

private:
    Ui::MoteConfig *ui;
    PacketInterface *mPacketInterface;

};

#endif // MOTECONFIG_H
