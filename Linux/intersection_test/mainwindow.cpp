#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mTimer = new QTimer(this);
    mTimer->start(20);

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
            car->setLocation(loc);
            ui->map->update();
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
    loc.setXY(0.0, -886.0);
    loc.setYaw(18.0 * (M_PI / 180.0));
    loc.setSpeed(20.0 / 3.6);
    car.setLocation(loc);
    ui->map->addCar(car);
}
