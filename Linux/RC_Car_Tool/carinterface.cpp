#include "carinterface.h"
#include "ui_carinterface.h"
#include "carinfo.h"
#include <QFileDialog>

CarInterface::CarInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CarInterface)
{
    ui->setupUi(this);

    // Plots
    ui->accelPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->gyroPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // The raw IMU plots
    maxSampleSize = 1000;
    accelXData.resize(maxSampleSize);
    accelYData.resize(maxSampleSize);
    accelZData.resize(maxSampleSize);
    gyroXData.resize(maxSampleSize);
    gyroYData.resize(maxSampleSize);
    gyroZData.resize(maxSampleSize);
    magXData.resize(maxSampleSize);
    magYData.resize(maxSampleSize);
    magZData.resize(maxSampleSize);
    accelGyroMagXAxis.resize(maxSampleSize);
    for(int i = 0;i < accelGyroMagXAxis.size();i++) {
        accelGyroMagXAxis[i] = (20.0 / 1000.0 * i);
    }

    mMap = 0;
    mId = 0;

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

CarInterface::~CarInterface()
{
    if (mMap) {
        mMap->removeCar(mId);
    }

    delete ui;
}

void CarInterface::setID(int id)
{
    ui->idBox->setValue(id);
}

int CarInterface::getId()
{
    return mId;
}

bool CarInterface::pollData()
{
    return ui->pollBox->isChecked();
}

void CarInterface::setOrientation(double roll, double pitch, double yaw)
{
    ui->rollBar->setValue(roll);
    ui->pitchBar->setValue(pitch);
    ui->yawBar->setValue(yaw);
    ui->orientationWidget->setRollPitchYaw(roll, pitch, yaw);
}

void CarInterface::setImuData(IMU_DATA data)
{
    accelXData.append(data.accel[0]);
    accelXData.remove(0, 1);
    accelYData.append(data.accel[1]);
    accelYData.remove(0, 1);
    accelZData.append(data.accel[2]);
    accelZData.remove(0, 1);

    ui->accelPlot->clearGraphs();
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::black));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelXData);
    ui->accelPlot->graph()->setName(tr("X"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::green));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelYData);
    ui->accelPlot->graph()->setName(tr("Y"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::blue));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelZData);
    ui->accelPlot->graph()->setName(tr("Z"));
    ui->accelPlot->rescaleAxes();
    ui->accelPlot->xAxis->setLabel("Seconds");
    ui->accelPlot->yAxis->setLabel("G");
    ui->accelPlot->legend->setVisible(true);
    ui->accelPlot->replot();

    gyroXData.append(data.gyro[0] * 180.0 / M_PI);
    gyroXData.remove(0, 1);
    gyroYData.append(data.gyro[1] * 180.0 / M_PI);
    gyroYData.remove(0, 1);
    gyroZData.append(data.gyro[2] * 180.0 / M_PI);
    gyroZData.remove(0, 1);

    ui->gyroPlot->clearGraphs();
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::black));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroXData);
    ui->gyroPlot->graph()->setName(tr("X"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::green));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroYData);
    ui->gyroPlot->graph()->setName(tr("Y"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::blue));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroZData);
    ui->gyroPlot->graph()->setName(tr("Z"));
    ui->gyroPlot->rescaleAxes();
    ui->gyroPlot->xAxis->setLabel("Seconds");
    ui->gyroPlot->yAxis->setLabel("deg/s");
    ui->gyroPlot->legend->setVisible(true);
    ui->gyroPlot->replot();

    magXData.append(data.mag[0]);
    magXData.remove(0, 1);
    magYData.append(data.mag[1]);
    magYData.remove(0, 1);
    magZData.append(data.mag[2]);
    magZData.remove(0, 1);

    ui->magPlot->clearGraphs();
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::black));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magXData);
    ui->magPlot->graph()->setName(tr("X"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::green));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magYData);
    ui->magPlot->graph()->setName(tr("Y"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::blue));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magZData);
    ui->magPlot->graph()->setName(tr("Z"));
    ui->magPlot->rescaleAxes();
    ui->magPlot->xAxis->setLabel("Seconds");
    ui->magPlot->yAxis->setLabel("uT");
    ui->magPlot->legend->setVisible(true);
    ui->magPlot->replot();

    setOrientation(data.roll, data.pitch, data.yaw);
    //ui->orientationWidget->setQuanternions(data.q[0], data.q[1], data.q[2], data.q[3]);
    //ui->rollBar->setValue(data.roll);
    //ui->pitchBar->setValue(data.pitch);
    //ui->yawBar->setValue(data.yaw);

    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        LocPoint loc = car->getLocation();
        loc.setAlpha(data.yaw * M_PI / 180.0);
        car->setLocation(loc);
        mMap->repaintAfterEvents();
    }

    QVector<double> magXYZ;
    magXYZ.append(data.mag[0]);
    magXYZ.append(data.mag[1]);
    magXYZ.append(data.mag[2]);

    if (ui->magSampleStoreBox->isChecked()) {
        mMagSamples.append(magXYZ);
    }
}

void CarInterface::setMap(MapWidget *map)
{
    mMap = map;
    CarInfo car(mId);
    mMap->addCar(car);
}

void CarInterface::timerSlot()
{
    // Update mag sample label
    static int lastMagSamples = 0;
    if (mMagSamples.size() != lastMagSamples) {
        ui->magSampleLabel->setText(QString::number(mMagSamples.size()) + " Samples");
    }
    lastMagSamples = mMagSamples.size();
}

void CarInterface::terminalPrint(quint8 id, QString str)
{
    if (id == mId) {
        ui->terminalBrowser->append(str);
    }
}

void CarInterface::vescFwdReceived(quint8 id, QByteArray data)
{

}

void CarInterface::on_terminalSendButton_clicked()
{
    emit terminalCmd(mId, ui->terminalEdit->text());
    ui->terminalEdit->clear();
}

void CarInterface::on_terminalClearButton_clicked()
{
    ui->terminalBrowser->clear();
}

void CarInterface::on_yawOffsetSlider_valueChanged(int value)
{
    ui->orientationWidget->setYawOffset(value);
}

void CarInterface::on_idBox_valueChanged(int arg1)
{
    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        car->setId(arg1, true);
    }

    mId = arg1;
}

void CarInterface::on_magSampleClearButton_clicked()
{
    mMagSamples.clear();
}

void CarInterface::on_magSampleSaveButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the magnetometer samples"));
    if (path.isNull()) {
        return;
    }

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    QVectorIterator<QVector<double> > i(mMagSamples);
    while (i.hasNext()) {
        QVector<double> element = i.next();
        out << element[0] << "\t" << element[1] << "\t" << element[2] << "\n";
    }

    file.close();
}
