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
#include "simscentree.h"

#include <QFileDialog>
#include <QDebug>
#include <QMenu>
#include <QAction>
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

//    Logger::Inst().SetCallback(log_callback);

    mScenarioEngine = 0;
    mOdrManager = 0;
    mScenarioGateway = 0;
    mSimTime = 0.0;
    mStory = new OscStory();

    mTimer = new QTimer(this);
    mTimer->start(20);

    ui->map->addMapModule(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    ui->mainSplitter->setStretchFactor(0, 1);
    ui->mainSplitter->setStretchFactor(1, 0);

    ui->scenTree->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->scenTree->header()->setStretchLastSection(false);
    ui->scenTree->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->scenTree, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(showScenTreeContextMenu(QPoint)));
}

PageSimScen::~PageSimScen()
{
    delete mScenarioEngine;
    delete mStory;

    mScenarioEngine = 0;
    mOdrManager = 0;

    delete ui;
}

void PageSimScen::updateScenTree()
{
    QList<QTreeWidgetItem*> items;

    for (OscAct &a: mStory->acts) {
        QTreeWidgetItem *it_a = new QTreeWidgetItem();
        it_a->setText(0, a.name);
        it_a->setData(0, Qt::UserRole, QVariant::fromValue(&a));
        items.append(it_a);

        for (OscSequence &s: a.sequences) {
            QTreeWidgetItem *it_s = new QTreeWidgetItem(it_a);
            it_s->setText(0, s.name);
            it_s->setData(0, Qt::UserRole, QVariant::fromValue(&s));

            for (OscManeuver &m: s.maneuvers) {
                QTreeWidgetItem *it_m = new QTreeWidgetItem(it_s);
                it_m->setText(0, m.name);
                it_m->setData(0, Qt::UserRole, QVariant::fromValue(&m));

                for (OscEvent &e: m.events) {
                    QTreeWidgetItem *it_e = new QTreeWidgetItem(it_m);
                    it_e->setText(0, e.name);
                    it_e->setData(0, Qt::UserRole, QVariant::fromValue(&e));

                    QTreeWidgetItem *it_ac = new QTreeWidgetItem(it_e);
                    it_ac->setText(0, e.action.name);
                    it_ac->setData(0, Qt::UserRole, QVariant::fromValue(&e.action));

                    for (OscCondition &c: e.conditions) {
                        QTreeWidgetItem *it_c = new QTreeWidgetItem(it_e);
                        it_c->setText(0, c.name);
                        it_c->setData(0, Qt::UserRole, QVariant::fromValue(&c));
                    }
                }
            }
        }
    }

    ui->scenTree->clear();
    ui->scenTree->insertTopLevelItems(0, items);
    ui->scenTree->expandAll();
}

void PageSimScen::selectNodeWithData(QVariant d)
{
    QTreeWidgetItemIterator it(ui->scenTree);
    while (*it) {
        // TODO: Can the pointer be extracted without all these tries?
        if ((*it)->data(0, Qt::UserRole) == d) {
            ui->scenTree->setCurrentItem(*it);
            break;
        }
        ++it;
    }
}

void PageSimScen::processPaint(QPainter &painter, int width, int height, bool highQuality,
                               QTransform drawTrans, QTransform txtTrans, double scale)
{
    (void)width; (void)height;
    (void)highQuality;

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

        // Plot selected scenario properties
        auto treeItem = ui->scenTree->currentItem();
        if (treeItem) {
            auto data = treeItem->data(0, Qt::UserRole);

            if (data.canConvert<OscCondition*>()) {
                auto c = data.value<OscCondition*>();
                if (c->type == OscCondByEntityReachPosLane) {
                    pos.SetLanePos(c->reachPosRoad, c->reachPosLane, c->reachPosS, 0);
                    painter.setTransform(drawTrans);
                    painter.setPen(Qt::NoPen);
                    painter.setBrush(Qt::blue);
                    painter.setOpacity(0.5);
                    QPointF center(pos.GetX() * 1000.0, pos.GetY() * 1000.0);
                    double rad = c->reachPosTolerance * 1000.0;
                    painter.drawEllipse(center, rad, rad);
                    painter.setOpacity(1.0);
                    auto ptTxt = drawTrans.map(center);
                    painter.setTransform(txtTrans);
                    painter.setBrush(Qt::red);
                    painter.drawEllipse(ptTxt, 5, 5);
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
            auto objState = mScenarioGateway->getObjectStatePtrByIdx(i)->state_;
            auto pos = objState.pos;

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
            LocPoint carAp = car->getApGoal();
            carAp.setRadius(0.0);
            car->setApGoal(carAp);
            car->setName(objState.name);
            car->setColor(QString(objState.name).toLower() == "ego" ? Qt::blue : Qt::red);
        }

        ui->map->update();
    }
}

void PageSimScen::showScenTreeContextMenu(const QPoint &pos)
{
    auto treeItem = ui->scenTree->itemAt(pos);
    if (treeItem) {
        QMenu contextMenu(ui->scenTree);
        QAction action1(treeItem->text(0), ui->scenTree);

        connect(&action1, &QAction::triggered, []() {

        });

        contextMenu.addAction(&action1);
        contextMenu.exec(ui->scenTree->mapToGlobal(pos));
    }
}

void PageSimScen::on_openScenarioButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load OpenScenario File"), "",
                                                    tr("OpenScenario files (*.xosc)"));

    if (!filename.isEmpty()) {
        mOscFileName = filename;
        mScenarioDoc.load_file(mOscFileName.toLocal8Bit().data());
        mStory->clear();
        mStory->load(mScenarioDoc.child("OpenSCENARIO").child("Storyboard").child("Story"));
        updateScenTree();
        on_restartButton_clicked();
    }
}

void PageSimScen::on_restartButton_clicked()
{
    if (mOscFileName.isEmpty()) {
        return;
    }

    mStory->save();
//        mScenarioDoc.save_file("/home/benjamin/Skrivbord/test222.xosc");

    ui->map->clearCars();

    try {
        if (mScenarioEngine) {
            delete mScenarioEngine;
        }

        mSimTime = 0.0;
        mScenarioEngine = new scenarioengine::ScenarioEngine(mScenarioDoc, mOscFileName.toStdString(), mSimTime,
                                                             scenarioengine::ExternalControlMode::EXT_CONTROL_BY_OSC);
        mScenarioGateway = mScenarioEngine->getScenarioGateway();
        mOdrManager = mScenarioEngine->getRoadManager();
    } catch (const std::exception& e) {
        mScenarioEngine = 0;
        mOdrManager = 0;
        mScenarioGateway = 0;
        qDebug() << "Could not open OSC file:" << e.what();
    }

    if (mScenarioEngine) {
        mScenarioEngine->step(0.0, true);
    }

    ui->map->update();
}

void PageSimScen::on_scenTree_currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
    (void)previous;

    ui->map->update();

    QWidget *w = ui->propArea->widget();
    if (w) {
        w->deleteLater();
    }

    if (!current) {
        return;
    }

    QVariant data = current->data(0, Qt::UserRole);

    auto reload = [data,this]() {
        // For some reason it crashes when not using a qtimer and reloading in
        // the next event loop iteration. TODO: Figure out why
        QTimer::singleShot(0, [data,this]() {
            updateScenTree();
            selectNodeWithData(data);
        });
    };

    if (data.canConvert<OscAct*>()) {
        OscAct* a = data.value<OscAct*>();
        QLineEdit *e = new QLineEdit();
        e->setText(a->name);
        ui->propArea->setWidget(e);

        connect(e, &QLineEdit::editingFinished, [this,a,e,reload]() {
            a->name = e->text();
            a->save();
            reload();
        });
    } else if (data.canConvert<OscSequence*>()) {
        OscSequence* s = data.value<OscSequence*>();
        QLineEdit *e = new QLineEdit();
        e->setText(s->name);
        ui->propArea->setWidget(e);

        connect(e, &QLineEdit::editingFinished, [this,s,e,reload]() {
            s->name = e->text();
            s->save();
            reload();
        });
    } else if (data.canConvert<OscEvent*>()) {
        OscEvent* e = data.value<OscEvent*>();
        (void)e;
    } else if (data.canConvert<OscCondition*>()) {
        OscCondition* c = data.value<OscCondition*>();
        if (c->type == OscCondByEntityReachPosLane) {
            QVBoxLayout *l = new QVBoxLayout;
            l->setSpacing(1);

            QLineEdit *nameEdit = new QLineEdit();
            nameEdit->setText(c->name);
            l->addWidget(nameEdit);

            QDoubleSpinBox *toleranceEdit = new QDoubleSpinBox;
            toleranceEdit->setPrefix("Tolerance: ");
            toleranceEdit->setDecimals(2);
            toleranceEdit->setSingleStep(0.2);
            toleranceEdit->setMaximum(999);
            toleranceEdit->setValue(c->reachPosTolerance);
            l->addWidget(toleranceEdit);

            QDoubleSpinBox *posEdit = new QDoubleSpinBox;
            posEdit->setPrefix("Pos: ");
            posEdit->setDecimals(1);
            posEdit->setMaximum(9999);
            posEdit->setValue(c->reachPosS);
            l->addWidget(posEdit);

            QSpinBox *roadEdit = new QSpinBox;
            roadEdit->setPrefix("Road: ");
            roadEdit->setMinimum(-999);
            roadEdit->setMaximum(999);
            roadEdit->setValue(c->reachPosRoad);
            l->addWidget(roadEdit);

            QSpinBox *laneEdit = new QSpinBox;
            laneEdit->setPrefix("Lane: ");
            laneEdit->setMinimum(-999);
            laneEdit->setMaximum(999);
            laneEdit->setValue(c->reachPosLane);
            l->addWidget(laneEdit);

            auto updateFunc = [this,c,nameEdit,toleranceEdit,posEdit,roadEdit,laneEdit,reload]() {
                bool nameChanged = c->name != nameEdit->text();
                c->name = nameEdit->text();
                c->reachPosTolerance = toleranceEdit->value();
                c->reachPosS = posEdit->value();
                c->reachPosRoad = roadEdit->value();
                c->reachPosLane = laneEdit->value();
                c->save();
                ui->map->update();
                if (nameChanged) {
                    reload();
                }
            };

            connect(nameEdit, &QLineEdit::editingFinished, updateFunc);
            connect(toleranceEdit, QOverload<double>::of(&QDoubleSpinBox::valueChanged), updateFunc);
            connect(posEdit, QOverload<double>::of(&QDoubleSpinBox::valueChanged), updateFunc);
            connect(roadEdit, QOverload<int>::of(&QSpinBox::valueChanged), updateFunc);
            connect(laneEdit, QOverload<int>::of(&QSpinBox::valueChanged), updateFunc);

            l->addStretch();
            QWidget *w = new QWidget;
            w->setLayout(l);
            ui->propArea->setWidget(w);
        }
    } else if (data.canConvert<OscAction*>()) {
        OscAction* a = data.value<OscAction*>();
        if (a->type == OscActionLatLaneChAbs) {
            QVBoxLayout *l = new QVBoxLayout;
            l->setSpacing(1);

            QLineEdit *nameEdit = new QLineEdit();
            nameEdit->setText(a->name);
            l->addWidget(nameEdit);

            QLineEdit *objectEdit = new QLineEdit();
            objectEdit->setText(a->latLaneChAbs_obj);
            l->addWidget(objectEdit);

            QSpinBox *valueEdit = new QSpinBox;
            valueEdit->setPrefix("Value: ");
            valueEdit->setMinimum(-999);
            valueEdit->setMaximum(999);
            valueEdit->setValue(a->latLaneChAbs_value);
            l->addWidget(valueEdit);

            QDoubleSpinBox *timeEdit = new QDoubleSpinBox;
            timeEdit->setPrefix("Time: ");
            timeEdit->setSuffix(" s");
            timeEdit->setDecimals(2);
            timeEdit->setMaximum(9999);
            timeEdit->setSingleStep(0.1);
            timeEdit->setValue(a->latLaneChAbs_time);
            l->addWidget(timeEdit);

            auto updateFunc = [this,a,nameEdit,objectEdit,valueEdit,timeEdit,reload]() {
                bool nameChanged = a->name != nameEdit->text();
                a->name = nameEdit->text();
                a->latLaneChAbs_obj = objectEdit->text();
                a->latLaneChAbs_value = valueEdit->value();
                a->latLaneChAbs_time = timeEdit->value();
                a->save();
                ui->map->update();
                if (nameChanged) {
                    reload();
                }
            };

            connect(nameEdit, &QLineEdit::editingFinished, updateFunc);
            connect(objectEdit, &QLineEdit::editingFinished, updateFunc);
            connect(valueEdit, QOverload<int>::of(&QSpinBox::valueChanged), updateFunc);
            connect(timeEdit, QOverload<double>::of(&QDoubleSpinBox::valueChanged), updateFunc);

            l->addStretch();
            QWidget *w = new QWidget;
            w->setLayout(l);
            ui->propArea->setWidget(w);
        }
    }
}
