#include "wireguard.h"
#include "ui_wireguard.h"
#include <QDebug>
#include <QMessageBox>

WireGuard::WireGuard(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WireGuard)
{
    ui->setupUi(this);
}

WireGuard::~WireGuard()
{
    delete ui;
}

QString WireGuard::genSimpleClientConfigWithKeys(QString clientIp, QString serverIp, int serverPort, QString serverPubKey)
{
    QString bashScript = QString(
        "#!/bin/sh\n"
        "privkey=$(wg genkey)\n"
        "pubkey=$(echo $privkey | wg pubkey)\n\n"

        "client_ip4=\"%1\"\n\n"

        "server_ip4=\"%2\"\n"
        "server_port=%3\n"
        "server_pubkey=\"%4\"\n\n"

        "printf \"" GENERATED_INFO_STRING "\n\n"
        "[Interface]\n "
        "# PublicKey = $pubkey \n"
        "PrivateKey = $privkey\n"
        "Address = $client_ip4/24\n\n"

        "[Peer]\n"
        "PublicKey = $server_pubkey\n"
        "Endpoint = $server_ip4:$server_port\n"
        "AllowedIPs = $client_ip4/24\n"
        "PersistentKeepalive = 25\n"
        "\n" GENERATED_INFO_STRING "\n\"\n")
        .arg(clientIp)
        .arg(serverIp)
        .arg(serverPort)
        .arg(serverPubKey);

    FILE *fp;
    if ((fp = popen(bashScript.toStdString().c_str(), "r")) == NULL) {
        return "# ERROR while generating configuration";
    }

    char buffer[1024];
    QString wg_conf;
    while (fgets((char*)&buffer, 1024, fp) != NULL)
        wg_conf += buffer;

    if(pclose(fp))
        return "# ERROR while generating configuration";

    return wg_conf;
}

QString WireGuard::getClientPubKeyFromConfig(QString config)
{
    QStringList confSplit = config.split(" ");
    // Assumption: first commented-out "PublicKey" is the client's PublicKey
    auto pubKey = std::find(confSplit.begin(), confSplit.end(), "PublicKey");
    if (*(pubKey-1) == "#")
        return *(pubKey+2);
    else
        return "";
}

bool WireGuard::priviledgedWriteConfigToFile(QString config, QString filename)
{
    QString writeConfigSh("echo '" + config + "' | pkexec bash -c 'tee " + filename + " && chmod 600 " + filename + "'");
    system(writeConfigSh.toStdString().c_str());

    // TODO evaluate pkexec return value
    return true;
}

QString WireGuard::priviledgedLoadConfigFromFile(QString filename)
{
    FILE *fp;
    QString bashScript = "pkexec cat " + filename;
    if ((fp = popen(bashScript.toStdString().c_str(), "r")) == NULL) {
        return "# ERROR while getting configuration";
    }

    char buffer[1024];
    QString wg_conf;
    while (fgets((char*)&buffer, 1024, fp) != NULL)
        wg_conf += buffer;

    if(pclose(fp))
        return "# ERROR while getting configuration";

    return wg_conf;
}

void WireGuard::on_WgGenConfigPushButton_clicked()
{
    ui->WgConfigPlainTextEdit->clear();
    ui->WgConfigPlainTextEdit->appendPlainText(
                WireGuard::genSimpleClientConfigWithKeys(ui->WgClientIPLineEdit->text(),
                                                         ui->WgServerIPLineEdit->text(), ui->WgServerPortSpinBox->value(), ui->WgServerPubKeyLineEdit->text()));
    ui->WgClientPubKeyLineEdit->setText(getClientPubKeyFromConfig(ui->WgConfigPlainTextEdit->toPlainText()));
}

void WireGuard::on_WgCancelPushButton_clicked()
{
    this->close();
}

void WireGuard::on_WgApplyConfigPushButton_clicked()
{
    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Information);
    if (priviledgedWriteConfigToFile(ui->WgConfigPlainTextEdit->toPlainText(), mConfFileName))
        msgBox.setText("Configuration was written to:\n" + mConfFileName);
    else
        msgBox.setText("Writing configuration failed.");
    msgBox.exec();
    this->close();
}

void WireGuard::on_WgLoadConfigPushButton_clicked()
{
    ui->WgConfigPlainTextEdit->clear();
    ui->WgConfigPlainTextEdit->appendPlainText(priviledgedLoadConfigFromFile(mConfFileName));
}
