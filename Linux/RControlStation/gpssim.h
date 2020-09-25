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

#ifndef GPSSIM_H
#define GPSSIM_H

#include <QDialog>
#include <QTimer>
#include "mapwidget.h"
#include "limesdr.h"

namespace Ui {
class GpsSim;
}

class GpsSim : public QDialog
{
    Q_OBJECT

public:
    explicit GpsSim(QWidget *parent = 0);
    ~GpsSim();
    void setMap(MapWidget *map);

private slots:
    void timerSlot();
    void statusUpdate(QString str);

    void on_startButton_clicked();
    void on_stopButton_clicked();
    void on_setPosButton_clicked();
    void on_clearButton_clicked();
    void on_simBaseBox_toggled(bool checked);

private:
    Ui::GpsSim *ui;
    MapWidget *mMap;
    LimeSDR *mSdr;
    QTimer *mTimer;

};

#endif // GPSSIM_H
