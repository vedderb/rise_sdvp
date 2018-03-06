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

#include "moteconfig.h"
#include "ui_moteconfig.h"
#include <QMessageBox>
#include <cstdio>

MoteConfig::MoteConfig(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MoteConfig)
{
    ui->setupUi(this);

    mPacketInterface = 0;

    ui->fixedLatBox->setValue(57.71495867);
    ui->fixedLonBox->setValue(12.89134921);
    ui->fixedHBox->setValue(219.0);
}

MoteConfig::~MoteConfig()
{
    delete ui;
}

void MoteConfig::setPacketInterface(PacketInterface *packetInterface)
{
    mPacketInterface = packetInterface;

    connect(mPacketInterface, SIGNAL(printReceived(quint8,QString)),
            this, SLOT(terminalPrint(quint8,QString)));
}

void MoteConfig::terminalPrint(quint8 id, QString str)
{
    if (id == ID_MOTE) {
        ui->terminalBrowser->append(str);
    }
}

void MoteConfig::on_setConfigButton_clicked()
{
    if (mPacketInterface) {
        int mode = 0;
        if (ui->modeOffButton->isChecked()) {
            mode = 0;
        } else if (ui->modeM8pSvinButton->isChecked()) {
            mode = 1;
        } else if (ui->modeM8pFixedButton->isChecked()) {
            mode = 2;
        } else if (ui->modeM8tSvinButton->isChecked()) {
            mode = 3;
        } else if (ui->modeM8tFixedButton->isChecked()) {
            mode = 4;
        }

        bool res = mPacketInterface->sendMoteUbxBase(mode,
                                                     ui->ubxFixedPosAccBox->value(),
                                                     ui->ubxSvinMinDurBox->value(),
                                                     ui->ubxSvinAccLimBox->value(),
                                                     ui->fixedLatBox->value(),
                                                     ui->fixedLonBox->value(),
                                                     ui->fixedHBox->value());

        if (!res) {
            QMessageBox::warning(this, "Mote Configuration",
                                 "Could not write configuration.");
        }
    }
}

void MoteConfig::on_terminalSendButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, ui->terminalEdit->text());
    }

    ui->terminalEdit->clear();
}

void MoteConfig::on_ubxPollNavSvinButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "ubx_poll UBX_NAV_SVIN");
    }
}

void MoteConfig::on_ubxPollNavRelPosNedButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "ubx_poll UBX_NAV_RELPOSNED");
    }
}

void MoteConfig::on_ubxPollRxmRawxButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "ubx_poll UBX_RXM_RAWX");
    }
}

void MoteConfig::on_ubxPollNavSolButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "ubx_poll UBX_NAV_SOL");
    }
}

void MoteConfig::on_m8tBaseStatusButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "base_status");
    }
}

void MoteConfig::on_m8tBaseResetButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->sendTerminalCmd(ID_MOTE, "base_reset");
    }
}
