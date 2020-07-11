#ifndef WIREGUARD_H
#define WIREGUARD_H

#include <QDialog>

#define GENERATED_INFO_STRING "##### GERERATED BY RCONTROLSTATION"

namespace Ui {
class WireGuard;
}

class WireGuard : public QDialog
{
    Q_OBJECT

public:
    explicit WireGuard(QWidget *parent = 0);
    ~WireGuard();

    static bool isWireGuardInstalled() {return system("which wg") == 0;}
    static bool isWireGuardSetup() {return isWireGuardInstalled() && system("pkexec test -f /etc/wireguard/wg_sdvp.conf") == 0; }
    static QString genSimpleClientConfigWithKeys(QString clientIp, QString serverIp, int serverPort, QString serverPubKey);
    static QString getClientPubKeyFromConfig(QString config);
    static bool priviledgedWriteConfigToFile(QString config, QString filename);
    static QString priviledgedLoadConfigFromFile(QString filename);

private slots:
    void on_WgGenConfigPushButton_clicked();

    void on_WgCancelPushButton_clicked();

    void on_WgApplyConfigPushButton_clicked();

    void on_WgLoadConfigPushButton_clicked();

private:
    Ui::WireGuard *ui;    
    QString mConfFileName = "/etc/wireguard/wg_sdvp.conf";
};

#endif // WIREGUARD_H
