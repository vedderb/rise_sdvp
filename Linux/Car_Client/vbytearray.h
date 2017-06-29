#ifndef VBYTEARRAY_H
#define VBYTEARRAY_H

#include <QByteArray>
#include <QString>

class VByteArray : public QByteArray
{
public:
    VByteArray();
    VByteArray(const QByteArray &data);

    void vbAppendInt32(qint32 number);
    void vbAppendUint32(quint32 number);
    void vbAppendInt16(qint16 number);
    void vbAppendUint16(quint16 number);
    void vbAppendInt8(qint8 number);
    void vbAppendUint8(quint8 number);
    void vbAppendDouble32(double number, double scale);
    void vbAppendDouble16(double number, double scale);
    void vbAppendDouble32Auto(double number);
    void vbAppendString(QString str);
    void vbAppendUint48(quint64 number);

    qint32 vbPopFrontInt32();
    quint32 vbPopFrontUint32();
    qint16 vbPopFrontInt16();
    quint16 vbPopFrontUint16();
    qint8 vbPopFrontInt8();
    quint8 vbPopFrontUint8();
    double vbPopFrontDouble32(double scale);
    double vbPopFrontDouble16(double scale);
    double vbPopFrontDouble32Auto();
    QString vbPopFrontString();
    quint64 vbPopFrontUint48();

};

#endif // VBYTEARRAY_H
