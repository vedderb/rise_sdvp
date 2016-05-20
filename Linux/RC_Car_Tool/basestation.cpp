#include "basestation.h"
#include "ui_basestation.h"
#include <QDebug>
#include <QMessageBox>
#include "nmeaserver.h"
#include "utility.h"

BaseStation::BaseStation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BaseStation)
{
    ui->setupUi(this);

    mTcpSocket = new QTcpSocket(this);
    mTcpConnected = false;
    mXNow = 0.0;
    mYNow = 0.0;
    mZNow = 0.0;
    mXAvg = 0.0;
    mYAvg = 0.0;
    mZAvg = 0.0;
    mAvgSamples = 0.0;

    mFixNowStr = "Solution...";
    mSatNowStr = "Sats...";

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));

    updateNmeaText();
}

BaseStation::~BaseStation()
{
    delete ui;
}

int BaseStation::getAvgPosLlh(double &lat, double &lon, double &height)
{
    double xAvg = mXAvg / mAvgSamples;
    double yAvg = mYAvg / mAvgSamples;
    double zAvg = mZAvg / mAvgSamples;

    utility::xyzToLlh(xAvg, yAvg, zAvg, &lat, &lon, &height);

    return (int)mAvgSamples;
}

void BaseStation::tcpInputConnected()
{
    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setText("Disconnect");
    mTcpConnected = true;
}

void BaseStation::tcpInputDisconnected()
{
    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setText("Connect");
    mTcpConnected = false;
}

void BaseStation::tcpInputDataAvailable()
{
    QByteArray nmea_msg =  mTcpSocket->readAll();

    NmeaServer::nmea_gga_info_t gga;
    QTextStream msgs(nmea_msg);

    while(!msgs.atEnd()) {
        QString str = msgs.readLine();
        QByteArray data = str.toLocal8Bit();

        // Hack
        if (str == "$GPGSA,A,1,,,,,,,,,,,,,,,*1E") {
            mFixNowStr = "Solution: Invalid";
            mSatNowStr = "Satellites: 0";
        }

        if (NmeaServer::decodeNmeaGGA(data, gga) >= 0) {
            mSatNowStr.sprintf("Satellites: %d", gga.n_sat);

            qDebug() << data;
            qDebug() << QString().sprintf("%.9f", gga.lat);

            switch (gga.fix_type) {
            case 0: mFixNowStr = "Solution: Invalid"; break;
            case 1: mFixNowStr = "Solution: SPP"; break;
            case 2: mFixNowStr = "Solution: DGPS"; break;
            case 3: mFixNowStr = "Solution: PPS"; break;
            case 4: mFixNowStr = "Solution: RTK Fix"; break;
            case 5: mFixNowStr = "Solution: RTK Float"; break;
            default: mFixNowStr = "Solution: Unknown"; break;
            }
        }

        if (gga.fix_type == 1 || gga.fix_type == 2 || gga.fix_type == 4 || gga.fix_type == 5) {
            utility::llhToXyz(gga.lat, gga.lon, gga.height, &mXNow, &mYNow, &mZNow);
            mXAvg += mXNow;
            mYAvg += mYNow;
            mZAvg += mZNow;
            mAvgSamples += 1.0;
        }

        updateNmeaText();
    }
}

void BaseStation::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "TcpError:" << errorStr;
    QMessageBox::warning(0, "BaseStation TCP Error", errorStr);

    mTcpSocket->close();

    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setText("Connect");
    mTcpConnected = false;
}

void BaseStation::on_nmeaConnectButton_clicked()
{
    if (mTcpConnected) {
        mTcpSocket->abort();

        ui->nmeaConnectButton->setEnabled(true);
        ui->nmeaConnectButton->setText("Connect");
        mTcpConnected = false;
    } else {
        mTcpSocket->abort();
        mTcpSocket->connectToHost(ui->nmeaServerEdit->text(), ui->nmeaPortBox->value());

        ui->nmeaConnectButton->setEnabled(false);
        ui->nmeaConnectButton->setText("Connecting...");
    }
}

void BaseStation::on_nmeaSampleClearButton_clicked()
{
    mXAvg = 0.0;
    mYAvg = 0.0;
    mZAvg = 0.0;
    mAvgSamples = 0.0;

    updateNmeaText();
}

void BaseStation::updateNmeaText()
{
    QString sampStr;
    sampStr.sprintf("Samples %d", (int)mAvgSamples);
    ui->nmeaSampleLabel->setText(sampStr);

    double xAvg = mXAvg / mAvgSamples;
    double yAvg = mYAvg / mAvgSamples;
    double zAvg = mZAvg / mAvgSamples;

    double lat_now, lon_now, height_now;
    double lat_avg, lon_avg, height_avg;

    utility::xyzToLlh(mXNow, mYNow, mZNow, &lat_now, &lon_now, &height_now);
    utility::xyzToLlh(xAvg, yAvg, zAvg, &lat_avg, &lon_avg, &height_avg);

    QString statStr;
    statStr += mFixNowStr + "\n" + mSatNowStr + "\n\n";
    statStr += QString().sprintf("XYZ Now: %.3f, %.3f, %.3f\n", mXNow, mYNow, mZNow);
    statStr += QString().sprintf("LLH Now: %.8f, %.8f, %.3f\n\n", lat_now, lon_now, height_now);

    statStr += QString().sprintf("XYZ Avg: %.3f, %.3f, %.3f\n", xAvg, yAvg, zAvg);
    statStr += QString().sprintf("LLH Avg: %.8f, %.8f, %.3f", lat_avg, lon_avg, height_avg);

    ui->nmeaBrowser->setText(statStr);
}
