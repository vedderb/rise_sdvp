/*
    Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "pagesimscen.h"
#include "ui_pagesimscen.h"

#include <QFileDialog>
#include <QDebug>
#include <cmath>
#include <clocale>

void log_callback(const char *str)
{
    qDebug() << str;
}

PageSimScen::PageSimScen(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PageSimScen)
{
    ui->setupUi(this);

    setlocale(LC_NUMERIC, "C");

    Logger::Inst().SetCallback(log_callback);

    mScenarioEngine = 0;
    mOdrManager = 0;
    mScenarioGateway = 0;
    mSimTime = 0.0;

    mTimer = new QTimer(this);
    mTimer->start(20);

    ui->map->addMapModule(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

PageSimScen::~PageSimScen()
{
    delete mScenarioEngine;

    mScenarioEngine = 0;
    mOdrManager = 0;

    delete ui;
}

void PageSimScen::processPaint(QPainter &painter, int width, int height, bool highQuality,
                               QTransform drawTrans, QTransform txtTrans, double scale)
{
    (void)painter;
    (void)width; (void)height;
    (void)highQuality;
    (void)drawTrans; (void)txtTrans;

    if (mOdrManager) {
        double step_length_target = 1.0;
        roadmanager::Position pos, pos_prev;
        QPen pen;

        for (int r = 0;r < mOdrManager->GetNumOfRoads();r++) {
            roadmanager::Road *road = mOdrManager->GetRoadByIdx(r);

            // Road Points?
            roadmanager::Geometry *geom = 0;
            for (int i = 0; i <= road->GetNumberOfGeometries(); i++) {
                if (i < road->GetNumberOfGeometries()) {
                    geom = road->GetGeometry(i);
                    pos.SetTrackPos(road->GetId(), geom->GetS(), 0);
                } else {
                    pos.SetTrackPos(road->GetId(), geom->GetS() + geom->GetLength(), 0);
                }

                painter.setTransform(drawTrans);
                painter.setBrush(Qt::red);
                pen.setWidth(0);
                painter.setPen(pen);
                painter.drawEllipse(QPointF(pos.GetX() * 1000.0, pos.GetY() * 1000.0), 6.0 / scale, 6.0 / scale);
            }

            // Lanes
            for (int i = 0; i < road->GetNumberOfLaneSections(); i++) {
                roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(i);
                double s_start = lane_section->GetS();
                double s_end = s_start + lane_section->GetLength();
                int steps = (int)((s_end - s_start) / step_length_target);
                double step_length = (s_end - s_start) / steps;

                for (int j = 0;j < lane_section->GetNumberOfLanes(); j++) {
                    roadmanager::Lane *lane = lane_section->GetLaneByIdx(j);

                    if (!lane->IsDriving() && lane->GetId() != 0) {
                        continue;
                    }

                    painter.setTransform(drawTrans);

                    if (lane->GetId() == 0) {
                        // Center Lane
                        pen.setWidthF(3.5 / scale);
                        pen.setColor(Qt::magenta);
                        painter.setPen(pen);
                    } else {
                        pen.setWidthF(2.5 / scale);
                        pen.setColor(Qt::green);
                        painter.setPen(pen);
                    }
    
                    for (int k = 0;k < steps + 1;k++) {
                        pos.SetLanePos(road->GetId(), lane->GetId(),
                                       fmin(s_end, s_start + (double)k * step_length), 0, i);

                        if (k > 0) {
                            painter.drawLine(pos_prev.GetX() * 1000.0, pos_prev.GetY() * 1000.0,
                                             pos.GetX() * 1000.0, pos.GetY() * 1000.0);
                        }

                        pos_prev = pos;
                    }
                }
            }
        }
    }
}

bool PageSimScen::processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                               QPoint widgetPos, LocPoint mapPos, double wheelAngleDelta,
                               bool ctrl, bool shift, bool ctrlShift,
                               bool leftButton, bool rightButton)
{
    (void)isPress; (void)isRelease; (void)isMove; (void)isWheel;
    (void)widgetPos; (void)mapPos;
    (void)wheelAngleDelta;
    (void)ctrl; (void)shift; (void)ctrlShift;
    (void)leftButton; (void)rightButton;

    return false;
}

void PageSimScen::timerSlot()
{
    double dt = (double)mTimer->interval() / 1000.0;
    mSimTime += dt;

    ui->map->setSelectedCar(ui->carBox->value());
    ui->map->setFollowCar(ui->followBox->isChecked() ? ui->carBox->value() : -1);
    ui->map->setDrawGrid(ui->drawGridBox->isChecked());

    if (mScenarioEngine && !ui->pauseButton->isChecked()) {
        mScenarioEngine->step(dt);

        for (int i = 0;i < mScenarioGateway->getNumberOfObjects();i++) {
            roadmanager::Position pos = mScenarioGateway->getObjectStatePtrByIdx(i)->state_.pos;

            CarInfo *car = ui->map->getCarInfo(i);
            if (!car) {
                CarInfo c(i);
                c.setLength(4.5);
                c.setWidth(2.0);
                c.setCornerRadius(0.1);
                ui->map->addCar(c);
                car = ui->map->getCarInfo(i);
            }

            LocPoint carPos = car->getLocation();
            carPos.setXY(pos.GetX(), pos.GetY());
            carPos.setYaw(-pos.GetH());
            car->setLocation(carPos);
        }

        ui->map->update();
    }
}

void PageSimScen::on_openScenarioButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load OpenScenario File"), "",
                                                    tr("OpenScenario files (*.xosc)"));

    if (!filename.isEmpty()) {
        mOscFileName = filename;
        on_restartButton_clicked();
    }
}

void PageSimScen::on_restartButton_clicked()
{
    if (mOscFileName.isEmpty()) {
        return;
    }

    try {
        if (mScenarioEngine) {
            delete mScenarioEngine;
        }

        mSimTime = 0.0;
        mScenarioEngine = new scenarioengine::ScenarioEngine(mOscFileName.toStdString(), mSimTime,
                                                             scenarioengine::ExternalControlMode::EXT_CONTROL_BY_OSC);
        mScenarioGateway = mScenarioEngine->getScenarioGateway();
        mOdrManager = mScenarioEngine->getRoadManager();
    } catch (const std::exception& e) {
        delete mScenarioEngine;
        mScenarioEngine = 0;
        mOdrManager = 0;
        mScenarioGateway = 0;
        qDebug() << "Could not open OSC file:" << e.what();
    }

    if (mScenarioEngine) {
        mScenarioEngine->step(0.0, true);
        ui->map->clearCars();
    }
}
