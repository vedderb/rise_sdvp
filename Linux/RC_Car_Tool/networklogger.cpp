#include "networklogger.h"
#include "ui_networklogger.h"
#include <QMessageBox>
#include <QDebug>
#include "nmeaserver.h"
#include "utility.h"
#include <cmath>
#include <QFileDialog>
#include <QStringList>
#include "qcustomplot.h"

NetworkLogger::NetworkLogger(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NetworkLogger)
{
    ui->setupUi(this);

    mTcpSocket = new QTcpSocket(this);
    mTcpConnected = false;
    mFixNowStr = "Solution...";
    mSatNowStr = "Sats...";
    mPing = new Ping(this);
    mMap = 0;

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
    connect(mPing, SIGNAL(pingRx(int,QString)), this, SLOT(pingRx(int,QString)));
    connect(mPing, SIGNAL(pingError(QString,QString)), this, SLOT(pingError(QString,QString)));

    ui->statHistogramPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

NetworkLogger::~NetworkLogger()
{
    if (mLog.isOpen()) {
        qDebug() << "Closing log:" << mLog.fileName();
        mLog.close();
    }

    delete ui;
}

void NetworkLogger::setMap(MapWidget *map)
{
    mMap = map;
}

void NetworkLogger::tcpInputConnected()
{
    ui->nmeaServerConnectButton->setEnabled(true);
    ui->nmeaServerConnectButton->setText("Disconnect");
    mTcpConnected = true;
}

void NetworkLogger::tcpInputDisconnected()
{
    ui->nmeaServerConnectButton->setEnabled(true);
    ui->nmeaServerConnectButton->setText("Connect");
    mTcpConnected = false;
}

void NetworkLogger::tcpInputDataAvailable()
{
    QByteArray nmea_msg =  mTcpSocket->readAll();

    NmeaServer::nmea_gga_info_t gga;
    QTextStream msgs(nmea_msg);
    bool gga_set = false;

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

            //qDebug() << data;
            //qDebug() << QString().sprintf("%.9f", gga.lat);

            switch (gga.fix_type) {
            case 0: mFixNowStr = "Solution: Invalid"; break;
            case 1: mFixNowStr = "Solution: SPP"; break;
            case 2: mFixNowStr = "Solution: DGPS"; break;
            case 3: mFixNowStr = "Solution: PPS"; break;
            case 4: mFixNowStr = "Solution: RTK Fix"; break;
            case 5: mFixNowStr = "Solution: RTK Float"; break;
            default: mFixNowStr = "Solution: Unknown"; break;
            }

            ui->nmeaFixTypeLabel->setText(mFixNowStr);
            ui->nmeaSatsLabel->setText(mSatNowStr);

            gga_set = true;
        }
    }

    if (gga_set && (gga.fix_type == 1 || gga.fix_type == 2 || gga.fix_type == 4 || gga.fix_type == 5)) {
        if (mPing->isRunning()) {
            // Return if previous ping is not finished yet
            return;
        }

        QString logStr;
        logStr.sprintf("%s    %.8f    %.8f    %.3f    %d    %d",
                       QDateTime::currentDateTime().toString(Qt::ISODate).toLocal8Bit().data(),
                       gga.lat, gga.lon, gga.height, gga.fix_type, ui->pingLenBox->value());
        bool pingOk = mPing->pingHost(ui->pingHostEdit->text(), ui->pingLenBox->value(), logStr);

        // Plot local position on map
        if (pingOk && mMap) {
            double i_llh[3];
            double llh[3];
            double xyz[3];

            mMap->getEnuRef(i_llh);
            llh[0] = gga.lat;
            llh[1] = gga.lon;
            llh[2] = gga.height;
            utility::llhToEnu(i_llh, llh, xyz);

            mLastPoint.setXY(xyz[0], xyz[1]);

            QString info;
            info.sprintf("Lat  : %.8f\n"
                         "Lon  : %.8f\n"
                         "H    : %.3f\n"
                         "Fix  : %d\n"
                         "Pktzs: %d\n",
                         gga.lat, gga.lon, gga.height, gga.fix_type, ui->pingLenBox->value());

            mLastPoint.setInfo(info);
        }
    }
}

void NetworkLogger::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "TcpError:" << errorStr;
    QMessageBox::warning(0, "BaseStation TCP Error", errorStr);

    mTcpSocket->close();

    ui->nmeaServerConnectButton->setEnabled(true);
    ui->nmeaServerConnectButton->setText("Connect");
    mTcpConnected = false;
}

void NetworkLogger::pingRx(int time, QString msg)
{
    QString msStr;
    msStr.sprintf("%.3f ms", (double)time / 1000.0);
    ui->pingMsLabel->setText(msStr);

    if (!msg.isEmpty()) {
        QString msStr;
        msStr.sprintf("    %.3f", (double)time / 1000.0);

        QString logLine;
        logLine += msg + msStr;
        if (ui->logPrintBox->isChecked()) {
            ui->logBrowser->append(logLine);
        }

        if (mLog.isOpen()) {
            mLog.write(QString(logLine + "\r\n").toLocal8Bit());
        }

        if (ui->plotMapBox->isChecked() && mMap) {
            QString info = mLastPoint.getInfo();
            msStr.sprintf("Time : %.3f ms", (double)time / 1000.0);
            info.append(msStr);
            mLastPoint.setInfo(info);
            mMap->addInfoPoint(mLastPoint);
        }
    }
}

void NetworkLogger::pingError(QString msg, QString error)
{
    ui->pingMsLabel->setText("error");

    if (msg.isEmpty()) {
        QMessageBox::warning(this, "Ping error", error);
    } else {
        QString logLine;
        logLine += msg + "    error";

        if (ui->logPrintBox->isChecked()) {
            ui->logBrowser->append(logLine);
        }

        if (mLog.isOpen()) {
            mLog.write(QString(logLine + "\r\n").toLocal8Bit());
        }

        if (ui->plotMapBox->isChecked() && mMap) {
            QString info = mLastPoint.getInfo();
            info.append("Time : error");
            mLastPoint.setInfo(info);
            mMap->addInfoPoint(mLastPoint);
        }
    }
}

void NetworkLogger::on_nmeaServerConnectButton_clicked()
{
    if (mTcpConnected) {
        mTcpSocket->abort();

        ui->nmeaServerConnectButton->setEnabled(true);
        ui->nmeaServerConnectButton->setText("Connect");
        mTcpConnected = false;
    } else {
        mTcpSocket->abort();
        mTcpSocket->connectToHost(ui->nmeaServerEdit->text(), ui->nmeaServerPortBox->value());

        ui->nmeaServerConnectButton->setEnabled(false);
        ui->nmeaServerConnectButton->setText("Connecting...");
    }
}

void NetworkLogger::on_pingTestButton_clicked()
{
    mPing->pingHost(ui->pingHostEdit->text(), ui->pingLenBox->value());
}

void NetworkLogger::on_logClearButton_clicked()
{
    ui->logBrowser->clear();
}

void NetworkLogger::on_logFileChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the network log"));
    if (path.isNull()) {
        return;
    }

    ui->logFileEdit->setText(path);
}

void NetworkLogger::on_logFileActiveBox_toggled(bool checked)
{
    if (checked) {
        if (mLog.isOpen()) {
            mLog.close();
        }

        mLog.setFileName(ui->logFileEdit->text());
        bool ok = mLog.open(QIODevice::ReadWrite | QIODevice::Append | QIODevice::Text);

        if (!ok) {
            QMessageBox::warning(this, "Log",
                                 "Could not open log file.");
            ui->logFileActiveBox->setChecked(false);
        }
    } else {
        mLog.close();
    }
}

void NetworkLogger::on_statLogOpenButton_clicked()
{
    QFile log;
    log.setFileName(ui->statLogLoadEdit->text());

    if (log.exists()) {
        bool ok = log.open(QIODevice::ReadOnly | QIODevice::Text);

        if (ok) {
            QTextStream in(&log);

            // Load log to memory
            mLogLoaded.clear();
            while(!in.atEnd()) {
                LOGPOINT lp;

                QString line = in.readLine();
                QStringList list = line.split(QRegExp("[ ]"), QString::SkipEmptyParts);

                lp.date = QDateTime::fromString(list.at(0), Qt::ISODate);
                lp.llh[0] = list.at(1).toDouble();
                lp.llh[1] = list.at(2).toDouble();
                lp.llh[2] = list.at(3).toDouble();
                lp.fix_type = list.at(4).toInt();
                lp.datalen = list.at(5).toInt();
                lp.pingRes = list.at(6);
                lp.pingMs = lp.pingRes.toDouble(&lp.pingOk);

                mLogLoaded.append(lp);
            }

            log.close();

            // Calculate statistics
            double ping_min = 1e100;
            double ping_max = 0.0;
            double ping_avg = 0.0;
            int ping_errors = 0;
            for (int i = 0;i < mLogLoaded.size();i++) {
                const LOGPOINT &lp = mLogLoaded.at(i);

                if (lp.pingOk) {
                    if (lp.pingMs < ping_min) {
                        ping_min = lp.pingMs;
                    }

                    if (lp.pingMs > ping_max) {
                        ping_max = lp.pingMs;
                    }

                    ping_avg += lp.pingMs;
                } else {
                    ping_errors++;
                }
            }

            ping_avg /= (double)mLogLoaded.size();

            QString statStr;
            statStr.sprintf("Samples : %d\n"
                            "Ping min: %.3f ms\n"
                            "Ping max: %.3f ms\n"
                            "Ping avg: %.3f ms\n"
                            "Ping err: %d (%.4f %%)",
                            mLogLoaded.size(), ping_min, ping_max,
                            ping_avg, ping_errors,
                            100.0 * (double)ping_errors / (double)mLogLoaded.size());
            ui->statBrowser->setText(statStr);

            // Plot on map
            if (mMap) {
                double i_llh[3];
                bool i_llh_set = false;
                for (int i = 0;i < mLogLoaded.size();i++) {
                    const LOGPOINT &lp = mLogLoaded.at(i);

                    if (!i_llh_set) {
                        if (ui->statLogZeroEnuBox->isChecked() || !mMap) {
                            i_llh[0] = lp.llh[0];
                            i_llh[1] = lp.llh[1];
                            i_llh[2] = lp.llh[2];
                        } else {
                            mMap->getEnuRef(i_llh);
                        }

                        i_llh_set = true;

                        if (mMap) {
                            mMap->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                        }
                    }

                    double xyz[3];
                    utility::llhToEnu(i_llh, lp.llh, xyz);

                    LocPoint p;
                    p.setXY(xyz[0], xyz[1]);

                    QString info;
                    info.sprintf("Date : %s\n"
                                 "Lat  : %.8f\n"
                                 "Lon  : %.8f\n"
                                 "H    : %.3f\n"
                                 "Fix  : %d\n"
                                 "Pktzs: %d\n"
                                 "Ping : %s",
                                 lp.date.toString(Qt::ISODate).toLocal8Bit().data(),
                                 lp.llh[0], lp.llh[1], lp.llh[2],
                            lp.fix_type, lp.datalen, lp.pingRes.toLocal8Bit().data());

                    p.setInfo(info);

                    const double l_green = ping_avg * 1.2;
                    const double l_red = ping_avg * 2.0;

                    if (lp.pingOk) {
                        if (lp.pingMs > l_red) {
                            p.setColor(Qt::red);
                        } else if (lp.pingMs > l_green) {
                            p.setColor(Qt::yellow);
                        }
                    } else {
                        p.setColor(Qt::red);
                    }

                    double w = 20.0;
                    if (lp.pingOk) {
                        w = utility::map(lp.pingMs, ping_min, ping_max, 3.0, 20.0);
                    }

                    p.setRadius(w);

                    mMap->addInfoPoint(p);
                }

                // Make histogram
                // create bottom axis rect for volume bar chart:
                QCustomPlot *customPlot = ui->statHistogramPlot;
                int n = 100;
                QDateTime start = QDateTime(QDate(2014, 6, 11));
                start.setTimeSpec(Qt::UTC);
                double startTime = 10.0;

                // create two bar plottables, for positive (green) and negative (red) volume bars:
                QCPBars *volumePos = new QCPBars(customPlot->xAxis, customPlot->yAxis);
                for (int i=0;i < n;i++) {
                    int v = qrand()%20000+qrand()%20000+qrand()%20000-10000*3;
                    volumePos->addData(startTime + 0.1 * i, qAbs(v)); // add data to either volumeNeg or volumePos, depending on sign of v
                }
                customPlot->setAutoAddPlottableToLegend(false);
                customPlot->addPlottable(volumePos);
                volumePos->setWidth(0.1 * 0.9);
                volumePos->setPen(Qt::NoPen);
                volumePos->setBrush(QColor(100, 180, 110));
                customPlot->rescaleAxes();
            }
        } else {
            QMessageBox::warning(this, "Open Error", "Could not open " + log.fileName());
        }

    } else {
        QMessageBox::warning(this, "Open Error", "Please select a valid log file");
    }
}

void NetworkLogger::on_statLogChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose log file to open"));
    if (path.isNull()) {
        return;
    }

    ui->statLogLoadEdit->setText(path);
}
