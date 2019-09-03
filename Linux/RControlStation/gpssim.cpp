/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#include "gpssim.h"
#include "ui_gpssim.h"
#include "utility.h"

GpsSim::GpsSim(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GpsSim)
{
    ui->setupUi(this);

    mSdr = new LimeSDR(this);
    mTimer = new QTimer(this);
    mTimer->start(20);
    mMap = 0;

    connect(mSdr, SIGNAL(statusUpdate(QString)),
            this, SLOT(statusUpdate(QString)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    ui->latBox->setValue(57.71495867);
    ui->lonBox->setValue(12.89134921);
    ui->heightBox->setValue(219.0);

    ui->latBaseBox->setValue(57.71495867);
    ui->lonBaseBox->setValue(12.89134921);
    ui->heightBaseBox->setValue(219.0);
}

GpsSim::~GpsSim()
{
    mSdr->stop();
    mSdr->wait(5000);
    delete ui;
}

void GpsSim::setMap(MapWidget *map)
{
    mMap = map;
}

void GpsSim::timerSlot()
{
    if (mMap && ui->followCarBox->isChecked()) {
        CarInfo *car = mMap->getCarInfo(ui->carBox->value());

        if (car) {
            double iLlh[3];
            double llh[3];
            double xyz[3];
            mMap->getEnuRef(iLlh);
            xyz[0] = car->getLocation().getX();
            xyz[1] = car->getLocation().getY();
            xyz[2] = car->getLocation().getHeight();
            utility::enuToLlh(iLlh, xyz, llh);
            ui->latBox->setValue(llh[0]);
            ui->lonBox->setValue(llh[1]);
            ui->heightBox->setValue(llh[2]);
            mSdr->setPos(ui->latBox->value(), ui->lonBox->value(), ui->heightBox->value(),
                         car->getLocation().getSpeed());
        }
    }
}

void GpsSim::statusUpdate(QString str)
{
    ui->textBrowser->moveCursor (QTextCursor::End);
    ui->textBrowser->insertPlainText (str);
    ui->textBrowser->moveCursor (QTextCursor::End);
}

void GpsSim::on_startButton_clicked()
{
    mSdr->setPosBase(ui->latBaseBox->value(),
                     ui->lonBaseBox->value(),
                     ui->heightBaseBox->value());
    mSdr->setBaseEnabled(ui->simBaseBox->isChecked());
    mSdr->start();
}

void GpsSim::on_stopButton_clicked()
{
    mSdr->stop();
}

void GpsSim::on_setPosButton_clicked()
{
    mSdr->setPos(ui->latBox->value(), ui->lonBox->value(), ui->heightBox->value(), 0.0);
}

void GpsSim::on_clearButton_clicked()
{
    ui->textBrowser->clear();
}

void GpsSim::on_simBaseBox_toggled(bool checked)
{
    ui->latBaseBox->setEnabled(checked);
    ui->lonBaseBox->setEnabled(checked);
    ui->heightBaseBox->setEnabled(checked);
}
