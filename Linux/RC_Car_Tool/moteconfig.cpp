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
        } else if (ui->modeUbxSvinButton->isChecked()) {
            mode = 1;
        } else if (ui->modeUbxFixedButton->isChecked()) {
            mode = 2;
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
