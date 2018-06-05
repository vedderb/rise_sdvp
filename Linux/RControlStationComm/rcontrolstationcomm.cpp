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

bool RControlStationComm::hasError()
{
    return mErrorMsgs.size() > 0;
}

char *RControlStationComm::lastError()
{
    if (hasError()) {
        QString text = mErrorMsgs.last().description;
        if (!mErrorMsgs.last().command.isEmpty()) {
            text.append(QString(" (cmd: %1)").arg(mErrorMsgs.last().command));
        }

        strcpy(mTextBuffer, text.toLocal8Bit().data());
        mErrorMsgs.removeLast();
    } else {
        strcpy(mTextBuffer, "No error");
    }

    return mTextBuffer;
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
                        stream.readElementText(); // Skip
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
                    } else if (name2 == "ap_route_left") {
                        state->ap_route_left = stream.readElementText().toInt();
                    } else if (name2 == "px_uwb") {
                        state->px_uwb = stream.readElementText().toDouble();
                    } else if (name2 == "py_uwb") {
                        state->py_uwb = stream.readElementText().toDouble();
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
        qWarning() << "libRControlStation: No response";
    }

    return ret;
}

bool RControlStationComm::getEnuRef(int car, bool fromMap, double *llh, int timeoutMs)
{
    bool ret = true;

    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        ret = false;
        return ret;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getEnuRef");
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("fromMap", QString::number(fromMap));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    QByteArray xml = waitForXml(timeoutMs);

    if (!xml.isEmpty()) {
        QXmlStreamReader stream(xml);
        stream.readNextStartElement();
        QString name;

        while (stream.readNextStartElement()) {
            if (stream.hasError()) {
                break;
            }

            name = stream.name().toString();

            if (name == "getEnuRef") {
                bool ok = true;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "id") {
                        stream.readElementText(); // Skip
                    } else if (name2 == "lat") {
                        llh[0] = stream.readElementText().toDouble();
                    } else if (name2 == "lon") {
                        llh[1] = stream.readElementText().toDouble();
                    } else if (name2 == "height") {
                        llh[2] = stream.readElementText().toDouble();
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
        qWarning() << "libRControlStation: No response";
    }

    return ret;
}

bool RControlStationComm::setEnuRef(int car, double *llh, int timeoutMs)
{
    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        return false;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("setEnuRef");
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("lat", QString::number(llh[0], 'g', 10));
    stream.writeTextElement("lon", QString::number(llh[1], 'g', 10));
    stream.writeTextElement("height", QString::number(llh[2]));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    return waitForAck("setEnuRef", timeoutMs);
}

bool RControlStationComm::addRoutePoints(int car, ROUTE_POINT *route, int len, bool replace,
                                         bool mapOnly, int mapRoute, int timeoutMs)
{
    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        return false;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    QString cmd = "addRoutePoint";
    if (replace) {
        cmd = "replaceRoute";
    }

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement(cmd);
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("mapOnly", QString::number(mapOnly));
    stream.writeTextElement("mapRoute", QString::number(mapRoute));

    for (int i = 0;i < len;i++) {
        stream.writeStartElement("point");
        stream.writeTextElement("px", QString::number(route[i].px));
        stream.writeTextElement("py", QString::number(route[i].py));
        stream.writeTextElement("speed", QString::number(route[i].speed));
        stream.writeTextElement("time", QString::number(route[i].time));
        stream.writeEndElement();
    }

    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    return waitForAck(cmd, timeoutMs);
}

bool RControlStationComm::clearRoute(int car, int mapRoute, int timeoutMs)
{
    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        return false;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    QString cmd = "clearRoute";
    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement(cmd);
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("mapRoute", QString::number(mapRoute));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    return waitForAck(cmd, timeoutMs);
}

bool RControlStationComm::setAutopilotActive(int car, bool active, int timeoutMs)
{
    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        return false;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    QString cmd = "setAutopilotActive";
    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement(cmd);
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("enabled", QString::number(active));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    return waitForAck(cmd, timeoutMs);
}

bool RControlStationComm::rcControl(int car, int mode, double value, double steering)
{
    bool ret = true;

    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        ret = false;
        return ret;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    QString cmd = "rcControl";
    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement(cmd);
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("mode", QString::number(mode));
    stream.writeTextElement("value", QString::number(value));
    stream.writeTextElement("steering", QString::number(steering));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());

    return ret;
}

bool RControlStationComm::getRoutePoints(int car, ROUTE_POINT *route, int *len,
                                         int maxLen, int mapRoute, int timeoutMs)
{
    bool ret = true;

    if (!isTcpConnected()) {
        qWarning() << "libRControlStation: not connected";
        ret = false;
        return ret;
    }

    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getRoute");
    stream.writeTextElement("id", QString::number(car));
    stream.writeTextElement("mapRoute", QString::number(mapRoute));
    stream.writeEndDocument();

    sendData(str.toLocal8Bit());
    QByteArray xml = waitForXml(timeoutMs);

    if (!xml.isEmpty()) {
        QXmlStreamReader stream(xml);
        stream.readNextStartElement();
        QString name;

        while (stream.readNextStartElement()) {
            if (stream.hasError()) {
                break;
            }

            name = stream.name().toString();

            if (name == "getRoute") {
                bool ok = true;
                int pointNow = 0;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "id") {
                        stream.readElementText(); // Skip
                    } else if (name2 == "point") {
                        ROUTE_POINT p;

                        while (stream.readNextStartElement()) {
                            QString name3 = stream.name().toString();

                            if (name3 == "px") {
                                p.px = stream.readElementText().toDouble();
                            } else if (name3 == "py") {
                                p.py = stream.readElementText().toDouble();
                            } else if (name3 == "speed") {
                                p.speed = stream.readElementText().toDouble();
                            } else if (name3 == "time") {
                                p.time = stream.readElementText().toInt();
                            } else {
                                qWarning() << "libRControlStation: argument not found:" << name3;
                                stream.skipCurrentElement();
                                ok = false;
                            }
                        }

                        if (pointNow < maxLen) {
                            route[pointNow++] = p;
                            *len = pointNow;
                        }
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
        qWarning() << "libRControlStation: No response";
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

        if (!checkError(xml)) {
            mXmlBuffer.append(xml);
        }

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

    while (mXmlBuffer.isEmpty() && !hasError()) {
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

bool RControlStationComm::waitForAck(QString cmd, int timeoutMs)
{
    bool ret = false;

    QString xml = waitForXml(timeoutMs);

    if (!xml.isEmpty()) {
        QXmlStreamReader stream(xml);
        stream.readNextStartElement();
        QString name;

        while (stream.readNextStartElement()) {
            if (stream.hasError()) {
                break;
            }

            name = stream.name().toString();

            if (name == "ack") {
                bool ok = true;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "command") {
                        if (stream.readElementText() == cmd) {
                            ret = true;
                        }
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
            }
        }

        if (stream.hasError()) {
            qWarning() << "libRControlStation: XML Parse error:" << stream.errorString();
        }
    } else {
        ret = false;
        qWarning() << "libRControlStation: No response";
    }

    if (!ret) {
        qWarning() << "libRControlStation: No ack received for command" << cmd;
    }

    return ret;
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

bool RControlStationComm::checkError(QString xml)
{
    bool ret = false;

    if (!xml.isEmpty()) {
        QXmlStreamReader stream(xml);
        stream.readNextStartElement();
        QString name;

        while (stream.readNextStartElement()) {
            if (stream.hasError()) {
                break;
            }

            name = stream.name().toString();

            if (name == "error") {
                bool ok = true;

                ERROR_MSG msg;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "command") {
                        msg.command = stream.readElementText();
                    } else if (name2 == "description") {
                        msg.description = stream.readElementText();
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

                mErrorMsgs.append(msg);
                while (mErrorMsgs.size() > 100) {
                    mErrorMsgs.remove(0);
                }

                qWarning() << "libRControlStation: XML error:" <<
                              msg.description << "(cmd:" << msg.command << ")";
                ret = true;
            }
        }

        if (stream.hasError()) {
            qWarning() << "libRControlStation: XML Parse error:" << stream.errorString();
        }
    }

    return ret;
}
