/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CONFCOMMONWIDGET_H
#define CONFCOMMONWIDGET_H

#include <QWidget>
#include <QVector>
#include "datatypes.h"

namespace Ui {
class ConfCommonWidget;
}

class ConfCommonWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ConfCommonWidget(QWidget *parent = 0);
    ~ConfCommonWidget();

    void getConfGui(MAIN_CONFIG &conf);
    void setConfGui(const MAIN_CONFIG &conf);

    void setMagComp(QVector<double> comp);
    void setMagCompCenter(QVector<double> center);

    void showAutoPilotConfiguration();

signals:
    void loadMagCal();

private slots:
    void on_magCalLoadButton_clicked();

    void on_confApResetOnEmergencyStopBox_toggled(bool checked);

private:
    Ui::ConfCommonWidget *ui;
};

#endif // CONFCOMMONWIDGET_H
