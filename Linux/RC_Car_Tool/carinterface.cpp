#include "carinterface.h"
#include "ui_carinterface.h"

CarInterface::CarInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CarInterface)
{
    ui->setupUi(this);
}

void CarInterface::setID(int id)
{
    ui->idBox->setValue(id);
}

int CarInterface::getId()
{
    return ui->idBox->value();
}

bool CarInterface::pollData()
{
    return ui->pollBox->isChecked();
}

void CarInterface::setOrientation(double roll, double pitch, double yaw)
{
    ui->orientationWidget->setRollPitchYaw(roll, pitch, yaw);
}

CarInterface::~CarInterface()
{
    delete ui;
}
