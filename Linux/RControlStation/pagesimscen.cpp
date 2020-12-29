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
#include <QInputDialog>
#include <QDebug>
#include <QMenu>
#include <QAction>
#include <QLabel>
#include <QSpacerItem>
#include <cmath>
#include <clocale>
#include <sstream>

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

    // Add default location for scenario files
    SE_AddPath("esmini/resources/xodr");
    SE_AddPath("esmini/resources/xosc");

    mOdrManager = 0;
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
    delete mOdrManager;
    delete mStory;

    mOdrManager = 0;

    delete ui;
}

void PageSimScen::updateScenTree()
{
    QList<QTreeWidgetItem*> items;
    mConditions.clear();

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
                        mConditions.append(&c);
                    }
                }
            }
        }
    }

    ui->scenTree->clear();
    ui->scenTree->insertTopLevelItems(0, items);
    ui->scenTree->expandAll();
    ui->map->update();
}

void PageSimScen::selectNodeWithData(QVariant d)
{
    QTreeWidgetItemIterator it(ui->scenTree);
    while (*it) {
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

                    painter.setTransform(drawTrans);

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

        // Draw text on top of lines (which is why we have to loop through everything again)
        for (int r = 0;r < mOdrManager->GetNumOfRoads();r++) {
            roadmanager::Road *road = mOdrManager->GetRoadByIdx(r);
            for (int i = 0; i < road->GetNumberOfLaneSections();i++) {
                roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(i);
                double s_start = lane_section->GetS();
                for (int j = 0;j < lane_section->GetNumberOfLanes();j++) {
                    roadmanager::Lane *lane = lane_section->GetLaneByIdx(j);

                    if (!lane->IsDriving() && lane->GetId() != 0) {
                        continue;
                    }

                    pos.SetLanePos(road->GetId(), lane->GetId(), s_start, 0, i);
                    QPointF center(pos.GetX() * 1000.0, pos.GetY() * 1000.0);
                    QPointF ptTxt = drawTrans.map(center);
                    ptTxt.setX(ptTxt.x() + 5);
                    ptTxt.setY(ptTxt.y() - 5);
                    painter.save();
                    painter.setTransform(txtTrans);
                    painter.setPen(Qt::black);
                    painter.drawText(ptTxt, QString("R%1:L%2").arg(road->GetId()).arg(lane->GetId()));
                    painter.restore();
                }
            }
        }

        OscCondition *condSel = 0;
        auto treeItem = ui->scenTree->currentItem();
        if (treeItem) {
            auto data = treeItem->data(0, Qt::UserRole);
            if (data.canConvert<OscCondition*>()) {
                condSel = data.value<OscCondition*>();
            }
        }

        for (auto c: mConditions) {
            if (c->type == OscCondByEntityReachPosLane) {
                pos.SetLanePos(c->reachPosRoad, c->reachPosLane, c->reachPosS, 0);
                painter.setTransform(drawTrans);
                painter.setBrush(Qt::blue);
                painter.setPen(Qt::NoPen);
                painter.setOpacity(0.5);
                QPointF center(pos.GetX() * 1000.0, pos.GetY() * 1000.0);
                double rad = c->reachPosTolerance * 1000.0;
                painter.drawEllipse(center, rad, rad);
                painter.setOpacity(1.0);
                auto ptTxt = drawTrans.map(center);
                painter.setTransform(txtTrans);
                if (c == condSel) {
                    painter.setBrush(Qt::red);
                } else {
                    painter.setBrush(Qt::gray);
                }
                painter.drawEllipse(ptTxt, 5, 5);
            }
        }
    }
}

bool PageSimScen::processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                               QPoint widgetPos, LocPoint mapPos, double wheelAngleDelta,
                               bool ctrl, bool shift, bool ctrlShift,
                               bool leftButton, bool rightButton, double scale)
{
    (void)isPress; (void)isRelease; (void)isMove; (void)isWheel;
    (void)widgetPos;
    (void)wheelAngleDelta;
    (void)ctrl; (void)shift; (void)ctrlShift;
    (void)leftButton; (void)rightButton;

    if (isPress) {
        roadmanager::Position pos;
        for (auto c: mConditions) {
            if (c->type == OscCondByEntityReachPosLane) {
                pos.SetLanePos(c->reachPosRoad, c->reachPosLane, c->reachPosS, 0);
                LocPoint posOnMap(pos.GetX(), pos.GetY());
                if (mapPos.getDistanceTo(posOnMap) < (1.0 / (scale * 100.0))) {
                    selectNodeWithData(QVariant::fromValue(c));
                    return true;
                }
            }
        }
    }

    return false;
}

void PageSimScen::timerSlot()
{
    double dt = (double)mTimer->interval() / 1000.0;
    mSimTime += dt;

    ui->map->setSelectedCar(ui->carBox->value());
    ui->map->setFollowCar(ui->followBox->isChecked() ? ui->carBox->value() : -1);
    ui->map->setDrawGrid(ui->drawGridBox->isChecked());

    if (!ui->pauseButton->isChecked()) {
        SE_StepDT(dt);

        for (int i = 0;i < SE_GetNumberOfObjects();i++) {
            SE_ScenarioObjectState objState;
            SE_GetObjectState(i, &objState);

            CarInfo *car = ui->map->getCarInfo(i);
            if (!car) {
                CarInfo c(i);
                c.setLength(objState.length);
                c.setWidth(objState.width);
                c.setCornerRadius(0.1);
                ui->map->addCar(c);
                car = ui->map->getCarInfo(i);
            }

            car->getLocation().setXY(objState.x, objState.y);
            car->getLocation().setYaw(-objState.h);
            car->getApGoal().setRadius(0.0);
            car->setName(SE_GetObjectName(i));
            car->setColor(i == 0 ? Qt::blue : Qt::red);
        }

        ui->map->update();
    }
}

void PageSimScen::showScenTreeContextMenu(const QPoint &pos)
{
    auto treeItem = ui->scenTree->itemAt(pos);
    if (treeItem) {
        auto data = treeItem->data(0, Qt::UserRole);

        if (data.canConvert<OscManeuver*>()) {
            auto m = data.value<OscManeuver*>();
            QMenu contextMenu(ui->scenTree);
            QAction actionAdd("Add Lane-Change Event", ui->scenTree);
            connect(&actionAdd, &QAction::triggered, [this,m,data]() {
                QString text = QInputDialog::getText(this, "Add lane-change event",
                                                     tr("Event name:"));
                if (!text.isEmpty()) {
                    m->addEventLaneChangePos(text);
                    updateScenTree();
                    selectNodeWithData(data);
                }
            });

            contextMenu.addAction(&actionAdd);
            contextMenu.exec(ui->scenTree->mapToGlobal(pos));
        } else if (data.canConvert<OscEvent*>()) {
            auto e = data.value<OscEvent*>();

            QMenu contextMenu(ui->scenTree);
            QAction actionDelete("Delete Event", ui->scenTree);
            connect(&actionDelete, &QAction::triggered, [this,e,data,treeItem]() {
                auto parentItem = treeItem->parent();
                if (parentItem) {
                    auto parantData = parentItem->data(0, Qt::UserRole);
                    if (parantData.canConvert<OscManeuver*>()) {
                        auto m = parantData.value<OscManeuver*>();
                        m->removeEvent(e->name);
                        updateScenTree();
                        selectNodeWithData(data);
                    }
                }
            });

            contextMenu.addAction(&actionDelete);
            contextMenu.exec(ui->scenTree->mapToGlobal(pos));
        }
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

void PageSimScen::on_saveScenarioButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save OpenScenario File"), "",
                                                    tr("OpenScenario files (*.xosc)"));

    if (!filename.isEmpty()) {
        if (!filename.toLower().endsWith(".xosc")) {
            filename.append(".xosc");
        }

        mScenarioDoc.save_file(filename.toLocal8Bit().data());
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
        std::stringstream ss;
        mScenarioDoc.save(ss, "  ");
        SE_InitWithString(ss.str().c_str(), 1, 0, 0, 0);
        mOdrManager = (roadmanager::OpenDrive*)SE_GetODRManager();
    } catch (const std::exception& e) {
        mOdrManager = 0;
        qDebug() << "Could not open OSC file:" << e.what();
    }

    SE_StepDT(0.0);

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

        QGridLayout *l = new QGridLayout;
        l->setVerticalSpacing(1);
        l->setHorizontalSpacing(4);
        l->setMargin(4);

        QLineEdit *e = new QLineEdit();
        e->setText(a->name);
        l->addWidget(new QLabel("Name"), 0, 0);
        l->addWidget(e, 0, 1);

        connect(e, &QLineEdit::editingFinished, [this,a,e,reload]() {
            a->name = e->text();
            a->save();
            reload();
        });

        l->addItem(new QSpacerItem(0, 0,
                                   QSizePolicy::MinimumExpanding,
                                   QSizePolicy::MinimumExpanding), 1, 1);
        QWidget *w = new QWidget;
        w->setLayout(l);
        ui->propArea->setWidget(w);
    } else if (data.canConvert<OscSequence*>()) {
        OscSequence* s = data.value<OscSequence*>();

        QGridLayout *l = new QGridLayout;
        l->setVerticalSpacing(1);
        l->setHorizontalSpacing(4);
        l->setMargin(4);

        QLineEdit *e = new QLineEdit();
        e->setText(s->name);
        l->addWidget(new QLabel("Name"), 0, 0);
        l->addWidget(e, 0, 1);

        connect(e, &QLineEdit::editingFinished, [this,s,e,reload]() {
            s->name = e->text();
            s->save();
            reload();
        });

        l->addItem(new QSpacerItem(0, 0,
                                   QSizePolicy::MinimumExpanding,
                                   QSizePolicy::MinimumExpanding), 1, 1);
        QWidget *w = new QWidget;
        w->setLayout(l);
        ui->propArea->setWidget(w);
    } else if (data.canConvert<OscManeuver*>()) {
        OscManeuver* m = data.value<OscManeuver*>();

        QGridLayout *l = new QGridLayout;
        l->setVerticalSpacing(1);
        l->setHorizontalSpacing(4);
        l->setMargin(4);

        QLineEdit *e = new QLineEdit();
        e->setText(m->name);
        l->addWidget(new QLabel("Name"), 0, 0);
        l->addWidget(e, 0, 1);

        connect(e, &QLineEdit::editingFinished, [this,m,e,reload]() {
            m->name = e->text();
            m->save();
            reload();
        });

        l->addItem(new QSpacerItem(0, 0,
                                   QSizePolicy::MinimumExpanding,
                                   QSizePolicy::MinimumExpanding), 1, 1);
        QWidget *w = new QWidget;
        w->setLayout(l);
        ui->propArea->setWidget(w);
    } else if (data.canConvert<OscEvent*>()) {
        OscEvent* ev = data.value<OscEvent*>();

        QGridLayout *l = new QGridLayout;
        l->setVerticalSpacing(1);
        l->setHorizontalSpacing(4);
        l->setMargin(4);

        QLineEdit *e = new QLineEdit();
        e->setText(ev->name);
        l->addWidget(new QLabel("Name"), 0, 0);
        l->addWidget(e, 0, 1);

        connect(e, &QLineEdit::editingFinished, [this,ev,e,reload]() {
            ev->name = e->text();
            ev->save();
            reload();
        });

        l->addItem(new QSpacerItem(0, 0,
                                   QSizePolicy::MinimumExpanding,
                                   QSizePolicy::MinimumExpanding), 1, 1);
        QWidget *w = new QWidget;
        w->setLayout(l);
        ui->propArea->setWidget(w);
    } else if (data.canConvert<OscCondition*>()) {
        OscCondition* c = data.value<OscCondition*>();
        if (c->type == OscCondByEntityReachPosLane) {
            QGridLayout *l = new QGridLayout;
            l->setVerticalSpacing(1);
            l->setHorizontalSpacing(4);
            l->setMargin(4);

            QLineEdit *nameEdit = new QLineEdit();
            nameEdit->setText(c->name);
            l->addWidget(new QLabel("Name"), 0, 0);
            l->addWidget(nameEdit, 0, 1);

            QLineEdit *entityNameEdit = new QLineEdit();
            entityNameEdit->setText(c->entityName);
            l->addWidget(new QLabel("Entity"), 1, 0);
            l->addWidget(entityNameEdit, 1, 1);

            QDoubleSpinBox *toleranceEdit = new QDoubleSpinBox;
            toleranceEdit->setPrefix("Tolerance: ");
            toleranceEdit->setDecimals(2);
            toleranceEdit->setSingleStep(0.2);
            toleranceEdit->setMaximum(999);
            toleranceEdit->setValue(c->reachPosTolerance);
            l->addWidget(toleranceEdit, 2, 0, 1, 2);

            QDoubleSpinBox *posEdit = new QDoubleSpinBox;
            posEdit->setPrefix("Pos: ");
            posEdit->setDecimals(1);
            posEdit->setMaximum(9999);
            posEdit->setValue(c->reachPosS);
            l->addWidget(posEdit, 3, 0, 1, 2);

            QSpinBox *roadEdit = new QSpinBox;
            roadEdit->setPrefix("Road: ");
            roadEdit->setMinimum(-999);
            roadEdit->setMaximum(999);
            roadEdit->setValue(c->reachPosRoad);
            l->addWidget(roadEdit, 4, 0, 1, 2);

            QSpinBox *laneEdit = new QSpinBox;
            laneEdit->setPrefix("Lane: ");
            laneEdit->setMinimum(-999);
            laneEdit->setMaximum(999);
            laneEdit->setValue(c->reachPosLane);
            l->addWidget(laneEdit, 5, 0, 1, 2);

            auto updateFunc = [this,c,nameEdit,entityNameEdit,
                    toleranceEdit,posEdit,roadEdit,laneEdit,reload]() {
                bool nameChanged = c->name != nameEdit->text();
                c->name = nameEdit->text();
                c->entityName = entityNameEdit->text();
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

            connect(entityNameEdit, &QLineEdit::editingFinished, updateFunc);
            connect(nameEdit, &QLineEdit::editingFinished, updateFunc);
            connect(toleranceEdit, QOverload<double>::of(&QDoubleSpinBox::valueChanged), updateFunc);
            connect(posEdit, QOverload<double>::of(&QDoubleSpinBox::valueChanged), updateFunc);
            connect(roadEdit, QOverload<int>::of(&QSpinBox::valueChanged), updateFunc);
            connect(laneEdit, QOverload<int>::of(&QSpinBox::valueChanged), updateFunc);

            l->addItem(new QSpacerItem(0, 0,
                                       QSizePolicy::Minimum,
                                       QSizePolicy::MinimumExpanding), 6, 0);
            QWidget *w = new QWidget;
            w->setLayout(l);
            ui->propArea->setWidget(w);
        }
    } else if (data.canConvert<OscAction*>()) {
        OscAction* a = data.value<OscAction*>();
        if (a->type == OscActionLatLaneChAbs) {
            QGridLayout *l = new QGridLayout;
            l->setVerticalSpacing(1);
            l->setHorizontalSpacing(4);
            l->setMargin(4);

            QLineEdit *nameEdit = new QLineEdit();
            nameEdit->setText(a->name);
            l->addWidget(new QLabel("Name"), 0, 0);
            l->addWidget(nameEdit, 0, 1);

            QLineEdit *objectEdit = new QLineEdit();
            objectEdit->setText(a->latLaneChAbs_obj);
            l->addWidget(new QLabel("Object"), 1, 0);
            l->addWidget(objectEdit, 1, 1);

            QSpinBox *valueEdit = new QSpinBox;
            valueEdit->setPrefix("Value: ");
            valueEdit->setMinimum(-999);
            valueEdit->setMaximum(999);
            valueEdit->setValue(a->latLaneChAbs_value);
            l->addWidget(valueEdit, 2, 0, 1, 2);

            QDoubleSpinBox *timeEdit = new QDoubleSpinBox;
            timeEdit->setPrefix("Time: ");
            timeEdit->setSuffix(" s");
            timeEdit->setDecimals(2);
            timeEdit->setMaximum(9999);
            timeEdit->setSingleStep(0.1);
            timeEdit->setValue(a->latLaneChAbs_time);
            l->addWidget(timeEdit, 3, 0, 1, 2);

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

            l->addItem(new QSpacerItem(0, 0,
                                       QSizePolicy::Minimum,
                                       QSizePolicy::MinimumExpanding), 4, 0);
            QWidget *w = new QWidget;
            w->setLayout(l);
            ui->propArea->setWidget(w);
        }
    }
}

void PageSimScen::on_drawOsmBox_toggled(bool checked)
{
    ui->map->setDrawOpenStreetmap(checked);
}
