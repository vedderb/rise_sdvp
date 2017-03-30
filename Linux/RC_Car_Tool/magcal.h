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

#ifndef MAGCAL_H
#define MAGCAL_H

#include <QWidget>

namespace Ui {
class MagCal;
}

class MagCal : public QWidget
{
    Q_OBJECT

public:
    explicit MagCal(QWidget *parent = 0);
    ~MagCal();

    void addSample(double x, double y, double z);
    bool calculateCompensation();

    double getCx();
    double getCy();
    double getCz();

    double getXx();
    double getXy();
    double getXz();

    double getYx();
    double getYy();
    double getYz();

    double getZx();
    double getZy();
    double getZz();

    QVector<double> getCenter();
    QVector<double> getComp();

private slots:
    void timerSlot();

    void on_magSampleClearButton_clicked();
    void on_magReplotButton_clicked();
    void on_magCalcCompButton_clicked();
    void on_magOpenFileButton_clicked();
    void on_magSampleSaveButton_clicked();
    void on_magCodeButton_clicked();

private:
    Ui::MagCal *ui;
    QTimer *mTimer;
    bool mMagReplot;

    QVector<QVector<double> > mMagSamples;
    QVector<double> mMagComp;
    QVector<double> mMagCompCenter;

    void loadMagPoints(QString path);
    void plotMagPoints();
    void calcMagComp();
    void updateMagPlots();
    void clearMagPlots();
};

#endif // MAGCAL_H
