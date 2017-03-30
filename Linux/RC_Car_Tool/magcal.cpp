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

#include "magcal.h"
#include "ui_magcal.h"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <QFileDialog>
#include <QMessageBox>
#include <cmath>

MagCal::MagCal(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MagCal)
{
    ui->setupUi(this);
    layout()->setContentsMargins(0, 0, 0, 0);

    ui->magSampXyPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magSampXzPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magSampYzPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->magSampXyPlot->yAxis->setLabel("XY");
    ui->magSampXyPlot->addGraph();
    ui->magSampXyPlot->graph()->setPen(QPen(Qt::red));
    ui->magSampXyPlot->graph()->setName("Uncompensated");
    ui->magSampXyPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampXyPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->magSampXyPlot->addGraph();
    ui->magSampXyPlot->graph()->setPen(QPen(Qt::blue));
    ui->magSampXyPlot->graph()->setName("Compensated");
    ui->magSampXyPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampXyPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));

    ui->magSampXzPlot->yAxis->setLabel("XZ");
    ui->magSampXzPlot->addGraph();
    ui->magSampXzPlot->graph()->setPen(QPen(Qt::red));
    ui->magSampXzPlot->graph()->setName("Uncompensated");
    ui->magSampXzPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampXzPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->magSampXzPlot->addGraph();
    ui->magSampXzPlot->graph()->setPen(QPen(Qt::blue));
    ui->magSampXzPlot->graph()->setName("Compensated");
    ui->magSampXzPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampXzPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));

    ui->magSampYzPlot->yAxis->setLabel("YZ");
    ui->magSampYzPlot->addGraph();
    ui->magSampYzPlot->graph()->setPen(QPen(Qt::red));
    ui->magSampYzPlot->graph()->setName("Uncompensated");
    ui->magSampYzPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampYzPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->magSampYzPlot->addGraph();
    ui->magSampYzPlot->graph()->setPen(QPen(Qt::blue));
    ui->magSampYzPlot->graph()->setName("Compensated");
    ui->magSampYzPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->magSampYzPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));

    mTimer = new QTimer(this);
    mTimer->start(20);
    mMagReplot = false;

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

MagCal::~MagCal()
{
    delete ui;
}

void MagCal::addSample(double x, double y, double z)
{
    if (ui->magSampleStoreBox->isChecked()) {
        QVector<double> magXYZ;
        magXYZ.append(x);
        magXYZ.append(y);
        magXYZ.append(z);
        mMagSamples.append(magXYZ);

        plotMagPoints();
    }
}

bool MagCal::calculateCompensation()
{
    bool res = false;

    if (mMagSamples.size() >= 9) {
        calcMagComp();
        res = true;
    }

    return res;
}

double MagCal::getCx()
{
    double res = -1;

    if (mMagCompCenter.size() == 3) {
       res = mMagCompCenter.at(0);
    }

    return res;
}

double MagCal::getCy()
{
    double res = -1;

    if (mMagCompCenter.size() == 3) {
       res = mMagCompCenter.at(1);
    }

    return res;
}

double MagCal::getCz()
{
    double res = -1;

    if (mMagCompCenter.size() == 3) {
       res = mMagCompCenter.at(2);
    }

    return res;
}

double MagCal::getXx()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(0);
    }

    return res;
}

double MagCal::getXy()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(1);
    }

    return res;
}

double MagCal::getXz()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(2);
    }

    return res;
}

double MagCal::getYx()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(3);
    }

    return res;
}

double MagCal::getYy()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(4);
    }

    return res;
}

double MagCal::getYz()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(5);
    }

    return res;
}

double MagCal::getZx()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(6);
    }

    return res;
}

double MagCal::getZy()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(7);
    }

    return res;
}

double MagCal::getZz()
{
    double res = -1;

    if (mMagComp.size() == 9) {
       res = mMagComp.at(8);
    }

    return res;
}

QVector<double> MagCal::getCenter()
{
    return mMagCompCenter;
}

QVector<double> MagCal::getComp()
{
    return mMagComp;
}

void MagCal::timerSlot()
{
    static int lastMagSamples = 0;
    if (mMagSamples.size() != lastMagSamples) {
        ui->magSampleLabel->setText(QString::number(mMagSamples.size()) + " Samples");
        lastMagSamples = mMagSamples.size();
    }

    if (mMagReplot) {
        updateMagPlots();
        mMagReplot = false;
    }
}

void MagCal::on_magSampleClearButton_clicked()
{
    mMagCompCenter.clear();
    mMagComp.clear();
    mMagSamples.clear();
    clearMagPlots();
}

void MagCal::on_magReplotButton_clicked()
{
    updateMagPlots();
}

void MagCal::on_magCalcCompButton_clicked()
{
    calcMagComp();
}

void MagCal::on_magOpenFileButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose magnetometer sample file."));
    if (path.isNull()) {
        return;
    }

    loadMagPoints(path);
}

void MagCal::on_magSampleSaveButton_clicked()
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

void MagCal::loadMagPoints(QString path)
{
    bool ok = true;
    QFile file(path);
    if (file.exists()) {
        mMagSamples.clear();

        if (file.open(QIODevice::ReadOnly)) {
            QTextStream in(&file);

            while (!in.atEnd()) {
                QString line = in.readLine();
                QStringList vals = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);

                if (vals.size() != 3) {
                    ok = false;
                    break;
                }

                bool readOk = false;
                QVector<double> magXYZ;

                magXYZ.append(vals.at(0).toDouble(&readOk));
                if (!readOk) {
                    ok = false;
                    break;
                }

                magXYZ.append(vals.at(1).toDouble(&readOk));
                if (!readOk) {
                    ok = false;
                    break;
                }

                magXYZ.append(vals.at(2).toDouble(&readOk));
                if (!readOk) {
                    ok = false;
                    break;
                }

                mMagSamples.append(magXYZ);
            }
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }

    if (!ok) {
        QMessageBox::warning(this, "Mag Cal",
                             "Could not load calibration file.");
    } else {
        plotMagPoints();
    }
}

void MagCal::plotMagPoints()
{
    QVector<double> magX, magY, magZ;

    for (int i = 0;i < mMagSamples.size();i++) {
        magX.append(mMagSamples.at(i).at(0));
        magY.append(mMagSamples.at(i).at(1));
        magZ.append(mMagSamples.at(i).at(2));
    }

    ui->magSampXyPlot->graph(0)->setData(magX, magY);
    ui->magSampXzPlot->graph(0)->setData(magX, magZ);
    ui->magSampYzPlot->graph(0)->setData(magY, magZ);

    mMagReplot = true;
}

void MagCal::calcMagComp()
{
    /*
     * Inspired by
     * http://davidegironi.blogspot.it/2013/01/magnetometer-calibration-helper-01-for.html#.UriTqkMjulM
     *
     * Ellipsoid fit from:
     * http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
     *
     * To use Eigen to convert matlab code, have a look at Eigen/AsciiQuickReference.txt
     */

    if (mMagSamples.size() < 9) {
        QMessageBox::warning(this, "Magnetometer compensation",
                             "Too few points.");
        return;
    }

    int samples = mMagSamples.size();
    Eigen::VectorXd ex(samples);
    Eigen::VectorXd ey(samples);
    Eigen::VectorXd ez(samples);

    for (int i = 0;i < samples;i++) {
        ex(i) = mMagSamples.at(i).at(0);
        ey(i) = mMagSamples.at(i).at(1);
        ez(i) = mMagSamples.at(i).at(2);
    }

    Eigen::MatrixXd eD(samples, 9);

    for (int i = 0;i < samples;i++) {
        eD(i, 0) = ex(i) * ex(i);
        eD(i, 1) = ey(i) * ey(i);
        eD(i, 2) = ez(i) * ez(i);
        eD(i, 3) = 2.0 * ex(i) * ey(i);
        eD(i, 4) = 2.0 * ex(i) * ez(i);
        eD(i, 5) = 2.0 * ey(i) * ez(i);
        eD(i, 6) = 2.0 * ex(i);
        eD(i, 7) = 2.0 * ey(i);
        eD(i, 8) = 2.0 * ez(i);
    }

    Eigen::MatrixXd etmp1 = eD.transpose() * eD;
    Eigen::MatrixXd etmp2 = eD.transpose() * Eigen::MatrixXd::Ones(samples, 1);
    Eigen::VectorXd eV = etmp1.lu().solve(etmp2);

    Eigen::MatrixXd eA(4, 4);
    eA(0,0)=eV(0);   eA(0,1)=eV(3);   eA(0,2)=eV(4);   eA(0,3)=eV(6);
    eA(1,0)=eV(3);   eA(1,1)=eV(1);   eA(1,2)=eV(5);   eA(1,3)=eV(7);
    eA(2,0)=eV(4);   eA(2,1)=eV(5);   eA(2,2)=eV(2);   eA(2,3)=eV(8);
    eA(3,0)=eV(6);   eA(3,1)=eV(7);   eA(3,2)=eV(8);   eA(3,3)=-1.0;

    Eigen::MatrixXd eCenter = -eA.topLeftCorner(3, 3).lu().solve(eV.segment(6, 3));
    Eigen::MatrixXd eT = Eigen::MatrixXd::Identity(4, 4);
    eT(3, 0) = eCenter(0);
    eT(3, 1) = eCenter(1);
    eT(3, 2) = eCenter(2);

    Eigen::MatrixXd eR = eT * eA * eT.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eEv(eR.topLeftCorner(3, 3) * (-1.0 / eR(3, 3)));
    Eigen::MatrixXd eVecs = eEv.eigenvectors();
    Eigen::MatrixXd eVals = eEv.eigenvalues();

    Eigen::MatrixXd eRadii(3, 1);
    eRadii(0) = sqrt(1.0 / eVals(0));
    eRadii(1) = sqrt(1.0 / eVals(1));
    eRadii(2) = sqrt(1.0 / eVals(2));

    Eigen::MatrixXd eScale = eRadii.asDiagonal().inverse() * eRadii.minCoeff();
    Eigen::MatrixXd eComp = eVecs * eScale * eVecs.transpose();

    mMagComp.resize(9);
    mMagComp[0] = eComp(0, 0);
    mMagComp[1] = eComp(0, 1);
    mMagComp[2] = eComp(0, 2);

    mMagComp[3] = eComp(1, 0);
    mMagComp[4] = eComp(1, 1);
    mMagComp[5] = eComp(1, 2);

    mMagComp[6] = eComp(2, 0);
    mMagComp[7] = eComp(2, 1);
    mMagComp[8] = eComp(2, 2);

    mMagCompCenter.resize(3);
    mMagCompCenter[0] = eCenter(0, 0);
    mMagCompCenter[1] = eCenter(1, 0);
    mMagCompCenter[2] = eCenter(2, 0);

    QVector<double> magX, magY, magZ;

    for (int i = 0;i < mMagSamples.size();i++) {
        double mx = mMagSamples.at(i).at(0);
        double my = mMagSamples.at(i).at(1);
        double mz = mMagSamples.at(i).at(2);

        mx -= mMagCompCenter.at(0);
        my -= mMagCompCenter.at(1);
        mz -= mMagCompCenter.at(2);

        magX.append(mx * mMagComp.at(0) + my * mMagComp.at(1) + mz * mMagComp.at(2));
        magY.append(mx * mMagComp.at(3) + my * mMagComp.at(4) + mz * mMagComp.at(5));
        magZ.append(mx * mMagComp.at(6) + my * mMagComp.at(7) + mz * mMagComp.at(8));
    }

    ui->magSampXyPlot->graph(1)->setData(magX, magY);
    ui->magSampXzPlot->graph(1)->setData(magX, magZ);
    ui->magSampYzPlot->graph(1)->setData(magY, magZ);

    updateMagPlots();
}

void MagCal::updateMagPlots()
{
    ui->magSampXyPlot->rescaleAxes();
    double xs = ui->magSampXyPlot->xAxis->range().size() / ui->magSampXyPlot->width();
    double ys = ui->magSampXyPlot->yAxis->range().size() / ui->magSampXyPlot->height();
    if (ys > xs) {
        ui->magSampXyPlot->xAxis->setScaleRatio(ui->magSampXyPlot->yAxis);
    } else {
        ui->magSampXyPlot->yAxis->setScaleRatio(ui->magSampXyPlot->xAxis);
    }
    ui->magSampXyPlot->replot();

    ui->magSampXzPlot->rescaleAxes();
    xs = ui->magSampXzPlot->xAxis->range().size() / ui->magSampXzPlot->width();
    ys = ui->magSampXzPlot->yAxis->range().size() / ui->magSampXzPlot->height();
    if (ys > xs) {
        ui->magSampXzPlot->xAxis->setScaleRatio(ui->magSampXzPlot->yAxis);
    } else {
        ui->magSampXzPlot->yAxis->setScaleRatio(ui->magSampXzPlot->xAxis);
    }
    ui->magSampXzPlot->replot();

    ui->magSampYzPlot->rescaleAxes();
    xs = ui->magSampYzPlot->xAxis->range().size() / ui->magSampYzPlot->width();
    ys = ui->magSampYzPlot->yAxis->range().size() / ui->magSampYzPlot->height();
    if (ys > xs) {
        ui->magSampYzPlot->xAxis->setScaleRatio(ui->magSampYzPlot->yAxis);
    } else {
        ui->magSampYzPlot->yAxis->setScaleRatio(ui->magSampYzPlot->xAxis);
    }
    ui->magSampYzPlot->replot();
}

void MagCal::clearMagPlots()
{
    QVector<double> x, y;

    ui->magSampXyPlot->graph(0)->setData(x, y);
    ui->magSampXzPlot->graph(0)->setData(x, y);
    ui->magSampYzPlot->graph(0)->setData(x, y);
    ui->magSampXyPlot->graph(1)->setData(x, y);
    ui->magSampXzPlot->graph(1)->setData(x, y);
    ui->magSampYzPlot->graph(1)->setData(x, y);

    ui->magSampXyPlot->replot();
    ui->magSampXzPlot->replot();
    ui->magSampYzPlot->replot();
}

void MagCal::on_magCodeButton_clicked()
{
    if (calculateCompensation()) {
        QString str;
        str.sprintf("conf->mag_cal_cx = %.4f;\n"
                    "conf->mag_cal_cy = %.4f;\n"
                    "conf->mag_cal_cz = %.4f;\n"
                    "\n"
                    "conf->mag_cal_xx = %.4f;\n"
                    "conf->mag_cal_xy = %.4f;\n"
                    "conf->mag_cal_xz = %.4f;\n"
                    "\n"
                    "conf->mag_cal_yx = %.4f;\n"
                    "conf->mag_cal_yy = %.4f;\n"
                    "conf->mag_cal_yz = %.4f;\n"
                    "\n"
                    "conf->mag_cal_zx = %.4f;\n"
                    "conf->mag_cal_zy = %.4f;\n"
                    "conf->mag_cal_zz = %.4f;\n",
                    mMagCompCenter.at(0), mMagCompCenter.at(1), mMagCompCenter.at(2),
                    mMagComp.at(0), mMagComp.at(1), mMagComp.at(2),
                    mMagComp.at(3), mMagComp.at(4), mMagComp.at(5),
                    mMagComp.at(6), mMagComp.at(7), mMagComp.at(8));

        QMessageBox::information(this, "Compensation Code", str);
    } else {
        QMessageBox::warning(this, "Show Code",
                             "Not enough samples to calculate compensation.");
    }
}
