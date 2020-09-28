#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utility.h"

#include <cmath>

namespace
{
int32_t get_I3(uint8_t *msg, int *ind) {
    int32_t res =	((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind + 0]);
    *ind += 3;

    if (res & 0b00000000100000000000000000000000) {
        res |= 0xFF000000;
    }

    return res;
}

float get_R4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind + 0]);
    *ind += 4;

    union asd {
        float f;
        uint32_t i;
    } x;

    x.i = res;

    return x.f;
}

double get_R8(uint8_t *msg, int *ind) {
    uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
                    ((uint64_t) msg[*ind + 6]) << 48 |
                    ((uint64_t) msg[*ind + 5]) << 40 |
                    ((uint64_t) msg[*ind + 4]) << 32 |
                    ((uint64_t) msg[*ind + 3]) << 24 |
                    ((uint64_t) msg[*ind + 2]) << 16 |
                    ((uint64_t) msg[*ind + 1]) << 8 |
                    ((uint64_t) msg[*ind + 0]);
    *ind += 8;

    union asd {
        double f;
        uint64_t i;
    } x;

    x.i = res;

    return x.f;
}

void put_I3(uint8_t *msg, int *ind, int32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
}

void put_R4(uint8_t *msg, int *ind, float data) {
    union asd {
        float f;
        uint32_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
}

void put_R8(uint8_t *msg, int *ind, double data) {
    union asd {
        double f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
    msg[(*ind)++] = x.i >> 32;
    msg[(*ind)++] = x.i >> 40;
    msg[(*ind)++] = x.i >> 48;
    msg[(*ind)++] = x.i >> 56;
}
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mTimer = new QTimer(this);
    mTimer->start(20);
    mUdpSocket = new QUdpSocket(this);

    // Center map at city area
    ui->map->setXOffset(-4500.0);
    ui->map->setYOffset(90000.0);
    ui->map->setScaleFactor(0.005);

    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));

    on_resetSim_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timerSlot()
{
    if (ui->runSim->text() == "Stop simulation") {
        CarInfo *car = ui->map->getCarInfo(0);

        if (car) {
            LocPoint loc = car->getLocation();

            double x = loc.getX();
            double y = loc.getY();
            double yaw = loc.getYaw();
            double dist = loc.getSpeed() * (double)mTimer->interval() * 1e-3;

            x += dist * cos(yaw);
            y -= dist * sin(yaw);

            loc.setXY(x, y);
            loc.setSpeed(ui->speedBox->value() / 3.6);
            car->setLocation(loc);
            ui->map->update();

            LocPoint locCenter;
            locCenter.setXY(43.75, -900.215);
            if (locCenter.getDistanceTo(loc) > fabs(ui->posBox->value()) * 1.1) {
                on_resetSim_clicked();
            }

            double illh[3];
            ui->map->getEnuRef(illh);

            sendNcom(illh, x, y, 0, yaw, loc.getSpeed());
        }
    }
}

void MainWindow::on_runSim_clicked()
{
    if (ui->runSim->text() == "Stop simulation") {
        ui->runSim->setText("Run simulation");
    } else {
        ui->runSim->setText("Stop simulation");
    }
}

void MainWindow::on_resetSim_clicked()
{
    ui->map->removeCar(0);

    // Add simulated car
    CarInfo car;
    LocPoint loc;
    loc.setXY(cos(18.0 * M_PI / 180.0) * ui->posBox->value() + 43.75,
              -sin(18.0 * M_PI / 180.0) * ui->posBox->value() - 900.215);
    loc.setYaw(18.0 * (M_PI / 180.0));
    loc.setSpeed(ui->speedBox->value() / 3.6);
    car.setLocation(loc);
    ui->map->addCar(car);
}

void MainWindow::sendNcom(double *illh,
                          double px,
                          double py,
                          double pz,
                          double heading,
                          double vel)
{
    double llh[3], xyz[3];

    xyz[0] = px;
    xyz[1] = py;
    xyz[2] = pz;

    utility::enuToLlh(illh, xyz, llh);
    double head = heading + (M_PI / 2.0);
    utility::norm_angle_rad(&head);

    double velN = vel * cos(head);
    double velE = vel * sin(head);
    double velD = 0.0;

    unsigned char packet[72];
    memset(packet, 0, sizeof(packet));

    int ind = 23;
    put_R8(packet, &ind, llh[0] * (M_PI / 180.0));
    put_R8(packet, &ind, llh[1] * (M_PI / 180.0));
    put_R4(packet, &ind, llh[2] * (M_PI / 180.0));
    ind = 43;
    put_I3(packet, &ind, (int32_t)(velN * 1e4));
    put_I3(packet, &ind, (int32_t)(velE * 1e4));
    put_I3(packet, &ind, (int32_t)(velD * 1e4));
    put_I3(packet, &ind, (int32_t)(head * 1e6));

    mUdpSocket->writeDatagram((char*)packet, sizeof(packet), QHostAddress::Broadcast, 3000);
}
