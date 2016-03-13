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

CarInterface::~CarInterface()
{
    delete ui;
}
