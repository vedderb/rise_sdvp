#include "rcontrolstationcomm.h"
#include <QDebug>
#include <QElapsedTimer>
#include <cstring>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

RControlStationComm::RControlStationComm()
{
    mApp = 0;
    mAppArgc = 1;
    mAppArgv[0] = "RControlStationComm";
    mAppArgv[1] = 0;

    if (QCoreApplication::instance() == 0) {
        mApp = new QCoreApplication(mAppArgc, (char**)mAppArgv);
    }

    mDebugLevel = 1;
    mTcpSocket = new QTcpSocket();
}

RControlStationComm::~RControlStationComm()
{
    delete mTcpSocket;

    if (mApp) {
        // Seems to crash bridj for some reason...
//        delete mApp;
//        mApp = 0;
    }
}

bool RControlStationComm::connectTcp(QString host, int port)
{
    mTcpSocket->abort();
    mTcpSocket->connectToHost(host, port);
    bool res = mTcpSocket->waitForConnected(1000);

    if (!res) {
        qWarning() << "libRControlStation: could not connect TCP:"
                   << mTcpSocket->errorString();
    } else if (mDebugLevel > 0) {
        qDebug() << "libRControlStation: TCP connected";
    }

    return res;
}

void RControlStationComm::disconnectTcp()
{
    mTcpSocket->abort();
    if (mDebugLevel > 0) {
        qDebug() << "libRControlStation: TCP disconnected";
    }
}

void RControlStationComm::setDebugLevel(int level)
{
    mDebugLevel = level;
}

bool RControlStationComm::getState(int car, CAR_STATE *state, int timeoutMs)
{
    bool ret = true;

    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        ret = false;
        return ret;
    }

    QByteArray xml = requestAnswer(car, "getState", timeoutMs);

    if (!xml.isEmpty()) {
        QXmlStreamReader stream(xml);
        stream.readNextStartElement();
        QString name;

        while (stream.readNextStartElement()) {
            if (stream.hasError()) {
                break;
            }

            name = stream.name().toString();

            if (name == "getState") {
                bool ok = true;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "id") {
                        stream.readElementText(); // Just skip it
                    } else if (name2 == "fw_major") {
                        state->fw_major = stream.readElementText().toInt();
                    } else if (name2 == "fw_minor") {
                        state->fw_minor = stream.readElementText().toInt();
                    } else if (name2 == "roll") {
                        state->roll = stream.readElementText().toDouble();
                    } else if (name2 == "pitch") {
                        state->pitch = stream.readElementText().toDouble();
                    } else if (name2 == "yaw") {
                        state->yaw = stream.readElementText().toDouble();
                    } else if (name2 == "accel_0") {
                        state->accel[0] = stream.readElementText().toDouble();
                    } else if (name2 == "accel_1") {
                        state->accel[1] = stream.readElementText().toDouble();
                    } else if (name2 == "accel_2") {
                        state->accel[2] = stream.readElementText().toDouble();
                    } else if (name2 == "gyro_0") {
                        state->gyro[0] = stream.readElementText().toDouble();
                    } else if (name2 == "gyro_1") {
                        state->gyro[1] = stream.readElementText().toDouble();
                    } else if (name2 == "gyro_2") {
                        state->gyro[2] = stream.readElementText().toDouble();
                    } else if (name2 == "mag_0") {
                        state->mag[0] = stream.readElementText().toDouble();
                    } else if (name2 == "mag_1") {
                        state->mag[1] = stream.readElementText().toDouble();
                    } else if (name2 == "mag_2") {
                        state->mag[2] = stream.readElementText().toDouble();
                    } else if (name2 == "px") {
                        state->px = stream.readElementText().toDouble();
                    } else if (name2 == "py") {
                        state->py = stream.readElementText().toDouble();
                    } else if (name2 == "speed") {
                        state->speed = stream.readElementText().toDouble();
                    } else if (name2 == "vin") {
                        state->vin = stream.readElementText().toDouble();
                    } else if (name2 == "temp_fet") {
                        state->temp_fet = stream.readElementText().toDouble();
                    } else if (name2 == "mc_fault") {
                        state->mc_fault = (mc_fault_code)stream.readElementText().toInt();
                    } else if (name2 == "px_gps") {
                        state->px_gps = stream.readElementText().toDouble();
                    } else if (name2 == "py_gps") {
                        state->py_gps = stream.readElementText().toDouble();
                    } else if (name2 == "ap_goal_px") {
                        state->ap_goal_px = stream.readElementText().toDouble();
                    } else if (name2 == "ap_goal_py") {
                        state->ap_goal_py = stream.readElementText().toDouble();
                    } else if (name2 == "ap_rad") {
                        state->ap_rad = stream.readElementText().toDouble();
                    } else if (name2 == "ms_today") {
                        state->ms_today = stream.readElementText().toInt();
                    } else {
                        qWarning() << "libRControlStation: argument not found:" << name2;
                        stream.skipCurrentElement();
                        ok = false;
                    }
                }

                if (stream.hasError()) {
                    break;
                }

                if (!ok) {
                    continue;
                }
            } else {
                qWarning() << "libRControlStation: Command not found: " << name;
                stream.skipCurrentElement();
            }
        }

        if (stream.hasError()) {
            qWarning() << "libRControlStation: XML Parse error:" << stream.errorString();
        }
    } else {
        ret = false;
        qWarning() << "libRControlStation: No response from car";
    }

    return ret;
}

void RControlStationComm::processData(const QByteArray &data)
{
    mRxBuffer.append(data);

    int start = mRxBuffer.indexOf("<message>");
    int end = mRxBuffer.indexOf("</message>");

    while (start >= 0 && end >= 0) {
        QByteArray xml = mRxBuffer.mid(start, end - start + 10);
        mRxBuffer.remove(start, end - start + 10);
        mXmlBuffer.append(xml);

        start = mRxBuffer.indexOf("<message>");
        end = mRxBuffer.indexOf("</message>");
    }

    // Clear buffer if it becomes too long
    if (mRxBuffer.size() > 5e6) {
        mRxBuffer.clear();
    }

    // Clear buffer if no message part is present
    if (mRxBuffer.size() > 9 && mRxBuffer.indexOf("<message>") < 0) {
        mRxBuffer.clear();
    }
}

void RControlStationComm::sendData(const QByteArray &data)
{
    if (isTcpConnected()) {
        mTcpSocket->write(data);
    }
}

QByteArray RControlStationComm::waitForXml(int timeoutMs)
{
    QByteArray res;
    QElapsedTimer t;
    t.start();

    while (mXmlBuffer.isEmpty()) {
        if (mTcpSocket->bytesAvailable() == 0) {
            mTcpSocket->waitForReadyRead(timeoutMs);
        }
        processData(mTcpSocket->readAll());

        if (t.elapsed() >= timeoutMs) {
            break;
        }
    }

    if (!mXmlBuffer.isEmpty()) {
        res = mXmlBuffer.first();
        mXmlBuffer.remove(0);
    }

    return res;
}

bool RControlStationComm::isTcpConnected()
{
    return mTcpSocket->state() == QTcpSocket::ConnectedState;
}

QByteArray RControlStationComm::requestAnswer(int car, QString cmd, int timeoutMs)
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement(cmd);
    stream.writeTextElement("id", QString::number(car));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    return waitForXml(timeoutMs);
}
