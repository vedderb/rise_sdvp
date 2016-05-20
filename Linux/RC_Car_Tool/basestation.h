#ifndef BASESTATION_H
#define BASESTATION_H

#include <QWidget>
#include <QTcpSocket>

namespace Ui {
class BaseStation;
}

class BaseStation : public QWidget
{
    Q_OBJECT

public:
    explicit BaseStation(QWidget *parent = 0);
    ~BaseStation();
    int getAvgPosLlh(double &lat, double &lon, double &height);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

    void on_nmeaConnectButton_clicked();
    void on_nmeaSampleClearButton_clicked();


private:
    Ui::BaseStation *ui;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;

    double mXNow;
    double mYNow;
    double mZNow;
    double mXAvg;
    double mYAvg;
    double mZAvg;
    double mAvgSamples;

    QString mFixNowStr;
    QString mSatNowStr;

    void updateNmeaText();
};

#endif // BASESTATION_H
