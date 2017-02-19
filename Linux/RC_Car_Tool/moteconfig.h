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

private:
    Ui::MoteConfig *ui;
    PacketInterface *mPacketInterface;

};

#endif // MOTECONFIG_H
