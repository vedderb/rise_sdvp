/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifdef HAS_GUI
#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#else
#include <QCoreApplication>
#endif

#include <QDebug>
#include <signal.h>
#include <QDir>
#include <QNetworkInterface>

#include "carclient.h"
#include "chronos.h"

void showHelp()
{
    qDebug() << "Arguments";
    qDebug() << "-h, --help : Show help text";
    qDebug() << "-p, --ttyport : Serial port, e.g. /dev/ttyUSB0";
    qDebug() << "-b, --baudrate : Serial baud rate, e.g. 9600";
    qDebug() << "-l, --log : Log to file, e.g. /tmp/logfile.bin (TODO!)";
    qDebug() << "--tcprtcmserver [port] : Start RTCM server on [port] (e.g. 8200)";
    qDebug() << "--tcpubxserver [port] : Start server for UBX data on [port] (e.g. 8210)";
    qDebug() << "--tcplogserver [port] : Start log server on [port] (e.g. 8410)";
    qDebug() << "--tcpnmeasrv : NMEA server address";
    qDebug() << "--tcpnmeaport : NMEA server port";
    qDebug() << "--useudp : Use UDP server";
    qDebug() << "--udpport : Port to use for the UDP server";
    qDebug() << "--usetcp : Use TCP server (will be used by default)";
    qDebug() << "--notcp : Do not use TCP server";
    qDebug() << "--tcpport : Specify port to use for the TCP server (default: 8300)";
    qDebug() << "--logusb : Store log files";
    qDebug() << "--logusbdir : Directory to store USB logs to";
    qDebug() << "--inputrtcm : Input RTCM data from serial port";
    qDebug() << "--ttyportrtcm : Serial port for RTCM, e.g. /dev/ttyUSB0";
    qDebug() << "--rtcmbaud : RTCM port baud rate, e.g. 9600";
    qDebug() << "--chronos : Run CHRONOS client";
    qDebug() << "--chronossettxid : Set CHRONOS transmitter id";
    qDebug() << "--chronoshostaddr [address] : Use network interface with address (default: any interface)";
    qDebug() << "--ntrip [server]:[stream]:[user]:[password]:[port] : Connect to ntrip server";
    qDebug() << "--rtcmbasepos [lat]:[lon]:[height] : Inject RTCM base position message";
    qDebug() << "--batterycells : Number of cells in series, e.g. for the GUI battery indicator";
    qDebug() << "--simulatecars [num]:[firstid] : Simulate num cars where the first car has ID firstid";
    qDebug() << "--dynosim : Run simulator together with output from AWITAR dyno instead of MotorSim";
    qDebug() << "--simaprepeatroutes [1 or 0] : Repeat routes setting for the simulation (default: 1)";
    qDebug() << "--simlogen [rateHz] : Enable simulator logging output at rateHz Hz";
    qDebug() << "--simuwben [port]:[rateHz] : Enable simulator UWB emulation output on TCP port port at rateHz Hz";
#ifdef HAS_GUI
    qDebug() << "--usegui : Use QML GUI";
#endif
}

static void m_cleanup(int sig)
{
    (void)sig;
    qApp->quit();
    qDebug() << "Bye :)";
}

int main(int argc, char *argv[])
{
#ifdef HAS_GUI
    QApplication a(argc, argv);
#else
    QCoreApplication a(argc, argv);
#endif

    QStringList args = QCoreApplication::arguments();
    QString ttyPort = "";
    QString logFile = "";
    int baudrate = 115200;
    int tcpRtcmPort = -1;
    int tcpUbxPort = -1;
    int tcpLogPort = -1;
    QString tcpNmeaServer = "127.0.0.1";
    int tcpNmeaPort = 2948;
    bool useUdp = false;
    int udpPort = 8300;
    bool useTcp = true;
    int tcpPort = 8300;
    bool logUsb = false;
    QString logUsbDir = QDir::currentPath() + "/logs";
    bool inputRtcm = false;
    QString ttyPortRtcm = "/dev/ttyUSB0";
    int rtcmBaud = 9600;
    bool useChronos = false;
    int chronosTxId = -1;
    QString chronosHostAddr;
    bool useNtrip = false;
    QString ntripServer;
    QString ntripStream;
    QString ntripUser;
    QString ntripPass;
    int ntripPort = 80;
    bool sendRtcmBase = false;
    double rtcmBaseLat = 0.0;
    double rtcmBaseLon = 0.0;
    double rtcmBaseHeight = 0.0;
    int batteryCells = 10;
    int simulateCarNum = 0;
    int simulateCarFirst = 0;
    bool dynoSim = false;
    bool simApRepeatRoutes = true;
    int simLogHz = -1;
    int simUwbHz = -1;
    int simUwbTcpPort = -1;

#ifdef HAS_GUI
    bool useGui = false;
#endif

    signal(SIGINT, m_cleanup);
    signal(SIGTERM, m_cleanup);

    for (int i = 0;i < args.size();i++) {
        // Skip the program argument
        if (i == 0) {
            continue;
        }

        QString str = args.at(i).toLower();

        // Skip path argument
        if (i >= args.size() && args.size() >= 3) {
            break;
        }

        bool dash = str.startsWith("-") && !str.startsWith("--");
        bool found = false;

        if ((dash && str.contains('h')) || str == "--help") {
            showHelp();
            found = true;
            return 0;
        }

        if ((dash && str.contains('p')) || str == "--ttyport") {
            if ((i + 1) < args.size()) {
                i++;
                ttyPort = args.at(i);
                found = true;
            }
        }

        if ((dash && str.contains('b')) || str == "--baudrate") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                baudrate = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if ((dash && str.contains('l')) || str == "--log") {
            if ((i + 1) < args.size()) {
                i++;
                logFile = args.at(i);
                found = true;
            }
        }

        if (str == "--tcprtcmserver") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                tcpRtcmPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--tcpubxserver") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                tcpUbxPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--tcplogserver") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                tcpLogPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--tcpnmeasrv") {
            if ((i + 1) < args.size()) {
                i++;
                tcpNmeaServer = args.at(i);
                found = true;
            }
        }

        if (str == "--tcpnmeaport") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                tcpNmeaPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--useudp") {
            useUdp = true;
            found = true;
        }

        if (str == "--udpport") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                udpPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--usetcp") {
            useTcp = true;
            found = true;
        }

        if (str == "--notcp") {
            useTcp = false;
            found = true;
        }

        if (str == "--tcpport") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                tcpPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--logusb") {
            logUsb = true;
            found = true;
        }

        if (str == "--logusbdir") {
            if ((i + 1) < args.size()) {
                i++;
                logUsbDir = args.at(i);
                found = true;
            }
        }

        if (str == "--inputrtcm") {
            inputRtcm = true;
            found = true;
        }

        if (str == "--ttyportrtcm") {
            if ((i + 1) < args.size()) {
                i++;
                ttyPortRtcm = args.at(i);
                found = true;
            }
        }

        if (str == "--rtcmbaud") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                rtcmBaud = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--chronos") {
            useChronos = true;
            found = true;
        }

        if (str == "--chronossettxid") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                chronosTxId = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--chronoshostaddr") {
            if ((i + 1) < args.size()) {
                i++;
                chronosHostAddr = args.at(i);
                found = true;
            }
        }

        if (str == "--ntrip") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList ntripData = tmp.split(":");

                if (ntripData.size() == 5) {
                    found = true;
                    ntripServer = ntripData.at(0);
                    ntripStream = ntripData.at(1);
                    ntripUser = ntripData.at(2);
                    ntripPass = ntripData.at(3);
                    ntripPort = ntripData.at(4).toInt();
                    useNtrip = true;
                }
            }
        }

        if (str == "--rtcmbasepos") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList baseData = tmp.split(":");

                if (baseData.size() == 3) {
                    found = true;
                    sendRtcmBase = true;
                    rtcmBaseLat = baseData.at(0).toDouble();
                    rtcmBaseLon = baseData.at(1).toDouble();
                    rtcmBaseHeight = baseData.at(2).toDouble();
                }
            }
        }

        if (str == "--batterycells") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                batteryCells = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--simulatecars") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList simData = tmp.split(":");

                if (simData.size() == 2) {
                    found = true;
                    simulateCarNum = simData.at(0).toInt();
                    simulateCarFirst = simData.at(1).toInt();
                }
            }
        }

        if (str == "--dynosim") {
            dynoSim = true;
            found = true;
        }

        if (str == "--simaprepeatroutes") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                simApRepeatRoutes = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--simlogen") {
            if ((i + 1) < args.size()) {
                i++;
                bool ok;
                simLogHz = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--simuwben") {
            if ((i + 1) < args.size()) {
                i++;
                QStringList param = args.at(i).split(":");
                if (param.size() == 2) {
                    found = true;
                    simUwbTcpPort = param.at(0).toInt();
                    simUwbHz = param.at(1).toInt();
                }
            }
        }

#ifdef HAS_GUI
        if (str == "--usegui") {
            useGui = true;
            found = true;
        }
#endif

        if (!found) {
            if (dash) {
                qCritical() << "At least one of the flags is invalid:" << str;
            } else {
                qCritical() << "Invalid option:" << str;
            }

            showHelp();
            return 1;
        }
    }

    CarClient car;
    Chronos chronos;
#ifdef HAS_GUI
    QQmlApplicationEngine qmlEngine;
#endif

    if (!ttyPort.isEmpty()) {
        car.connectSerial(ttyPort, baudrate);
    } else {
        // Not connected to any car, set default ID
        if (simulateCarNum > 0) {
            car.setCarId(simulateCarFirst);
        } else {
            car.setCarId(0);
        }
    }

    for (int i = 0;i < simulateCarNum;i++) {
        car.addSimulatedCar(i + simulateCarFirst);
        car.getSimulatedCar(i + simulateCarFirst)->autopilot()->setRepeatRoutes(simApRepeatRoutes);
        if (simLogHz > 0) {
            car.getSimulatedCar(i + simulateCarFirst)->startLogBroadcast(simLogHz);
        }
        if (simUwbHz > 0) {
            car.getSimulatedCar(i + simulateCarFirst)->startUwbBroadcast(simUwbTcpPort, simUwbHz);
        }
    }

    if (tcpRtcmPort >= 0) {
        car.startRtcmServer(tcpRtcmPort);
    }

    if (tcpUbxPort >= 0) {
        car.startUbxServer(tcpUbxPort);
    }

    if (tcpLogPort >= 0) {
        car.startLogServer(tcpLogPort);
    }

    car.restartRtklib();
    car.setBatteryCells(batteryCells);

    if (car.isRtklibRunning()) {
        car.connectNmea(tcpNmeaServer, tcpNmeaPort);
    }

    if (useUdp) {
        car.startUdpServer(udpPort);
    }

    if (useTcp) {
        if (chronosHostAddr.isNull()) {
            car.startTcpServer(tcpPort);
        } else {
            // Start Car Tcp server at same addr as chronos host addr
            car.startTcpServer(tcpPort,QHostAddress(chronosHostAddr));
        }
    }

    if (logUsb) {
        car.enableLogging(logUsbDir);
    }

    if (inputRtcm) {
        car.connectSerialRtcm(ttyPortRtcm, rtcmBaud);
    }

    if (useChronos) {
        qDebug() << "CHRONOS!!!";
        if (!chronosHostAddr.isEmpty()) {
            QHostAddress addr(chronosHostAddr);
            if (!addr.isNull()) {
                if (QNetworkInterface::allAddresses().contains(addr)) {
                    chronos.startServer(car.packetInterface(), addr);
                    chronos.comm()->setTransmitterId(addr.toIPv4Address() & 0xFF);
                } else {
                    chronos.startServer(car.packetInterface());
                    qWarning() << "There is no interface with address" <<
                                  chronosHostAddr;
                }
            } else {
                chronos.startServer(car.packetInterface());
                qWarning() << chronosHostAddr << "is not a valid address";
            }
        } else {
            chronos.startServer(car.packetInterface());
        }

        if (chronosTxId >= 0) {
            chronos.comm()->setTransmitterId(chronosTxId);
        }

        // In case we simulate, use CHRONOS-compatible settings
        CarSim *sim = car.getSimulatedCar(simulateCarFirst);
        if (sim) {
            sim->autopilot()->setModeTime(2);
            sim->autopilot()->setRepeatRoutes(false);
        }
    }

    if (useNtrip) {
        car.connectNtrip(ntripServer, ntripStream, ntripUser, ntripPass, ntripPort);
    }

    if (sendRtcmBase) {
        car.setSendRtcmBasePos(true, rtcmBaseLat, rtcmBaseLon, rtcmBaseHeight);
    }

    if (dynoSim) {
        CarSim *sim = car.getSimulatedCar(simulateCarFirst);
        if (sim) {
            sim->listenDyno();
            sim->autopilot()->setBaseRad(8.0);
            sim->setAxisDistance(3.0);
            sim->setCarTurnRad(6.0);
        }
    }

#ifdef HAS_GUI
    if (useGui) {
        qmlRegisterType<PacketInterface>("Car.packetInterface", 1, 0, "PacketInterface");
        qmlEngine.rootContext()->setContextProperty("carClient", &car);
        qmlEngine.load(QUrl(QLatin1String("qrc:/res/main.qml")));

        if (qmlEngine.rootObjects().isEmpty()) {
            qWarning() << "Could not start QML GUI";
        }
    }
#endif

    return a.exec();
}
