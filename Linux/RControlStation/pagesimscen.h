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

#ifndef PAGESIMSCEN_H
#define PAGESIMSCEN_H

#include <QWidget>
#include <QTimer>
#include <QDebug>
#include <QVector>

#include "mapwidget.h"

#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "pugixml.hpp"

namespace Ui {
class PageSimScen;
}

class PageSimScen : public QWidget, public MapModule
{
    Q_OBJECT

public:
    explicit PageSimScen(QWidget *parent = 0);
    ~PageSimScen();

    void processPaint(QPainter &painter, int width, int height, bool highQuality,
                      QTransform drawTrans, QTransform txtTrans, double scale);
    bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                      QPoint widgetPos, LocPoint mapPos, double wheelAngleDelta,
                      bool ctrl, bool shift, bool ctrlShift,
                      bool leftButton, bool rightButton);

private slots:
    void timerSlot();

    void on_openScenarioButton_clicked();
    void on_restartButton_clicked();

private:
    Ui::PageSimScen *ui;
    scenarioengine::ScenarioEngine *mScenarioEngine;
    roadmanager::OpenDrive *mOdrManager;
    scenarioengine::ScenarioGateway *mScenarioGateway;
    double mSimTime;
    QTimer *mTimer;
    QString mOscFileName;
    pugi::xml_document mScenarioDoc;

    typedef enum {
        OscActionLateralLaneChange = 0
    } OscActionType;

    class OscAction {
    public:
        QString name;
        OscActionType type;
        pugi::xml_node node;

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());
        }
    };

    typedef enum {
        OscCondUnknown = 0,
        OscCondByEntityTimeHead,
        OscCondByEntityReachPosLane,
        OscCondByStateStart
    } OscCondType;

    class OscCondition {
    public:
        QString name;
        OscCondType type;
        pugi::xml_node node;

        QString entityName;
        double reachPosTolerance;
        double reachPosS;
        int reachPosLane;
        int reachPosRoad;

        OscCondition() {}

        OscCondition(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();

            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "ByEntity") {
                    for (auto child2 = child.first_child(); child2; child2 = child2.next_sibling()) {
                        if (QString(child2.name()) == "TriggeringEntities") {
                            for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                                if (QString(child3.name()) == "Entity") {
                                    entityName = child3.attribute("name").as_string();
                                }
                            }
                        } else if (QString(child.name()) == "EntityCondition") {
                            for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                                if (QString(child3.name()) == "ReachPosition") {
                                    reachPosTolerance = child3.attribute("tolerance").as_double();
                                    for (auto child4 = child3.first_child(); child4; child4 = child4.next_sibling()) {
                                        if (QString(child4.name()) == "Position") {
                                            for (auto child5 = child4.first_child(); child5; child5 = child5.next_sibling()) {
                                                if (QString(child5.name()) == "Lane") {
                                                    type = OscCondByEntityReachPosLane;
                                                    reachPosRoad = child5.attribute("roadId").as_int();
                                                    reachPosLane = child5.attribute("laneId").as_int();
                                                    reachPosS = child5.attribute("s").as_double();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());

            switch (type) {
            case OscCondByEntityReachPosLane: {
                node.child("ByEntity").
                        child("TriggeringEntities").
                        child("Entity").attribute("name").
                        set_value(entityName.toLocal8Bit().data());
                node.child("ByEntity").
                        child("EntityCondition").
                        child("ReachPosition").
                        attribute("tolerance").
                        set_value(reachPosTolerance);

                pugi::xml_node lane = node.child("ByEntity").
                        child("EntityCondition").
                        child("ReachPosition").
                        child("Position").
                        child("Lane");

                lane.attribute("roadId").set_value(reachPosRoad);
                lane.attribute("laneId").set_value(reachPosLane);
                lane.attribute("s").set_value(reachPosS);
            } break;

            default:
                break;
            }
        }
    };

    class OscEvent {
    public:
        QString name;
        OscAction action;
        QVector<OscCondition> conditions;
        pugi::xml_node node;

        OscEvent() {}

        OscEvent(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();

            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "Action") {
                    action.load(child);
                } else if (QString(child.name()) == "StartConditions") {
                    for (auto child2 = child.first_child(); child2; child2 = child2.next_sibling()) {
                        if (QString(child2.name()) == "ConditionGroup") {
                            for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                                if (QString(child3.name()) == "Condition") {
                                    conditions.append(OscCondition(child3));
                                }
                            }
                        }
                    }
                }
            }
        }

        void save() {
            name = node.attribute("name").as_string();
            action.save();
            for (auto cond: conditions) {
                cond.save();
            }
        }
    };

    class OscManeuver {
    public:
        QString name;
        QVector<OscEvent> events;
        pugi::xml_node node;

        OscManeuver() {}

        OscManeuver(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        bool removeEvent(QString name) {
            for (int i = 0;i < events.size();i++) {
                if (events.at(i).name == name) {
                    node.remove_child(events.at(i).node);
                    events.remove(i);
                    return true;
                }
            }
            return false;
        }

        void addEvent(QString name) {
            pugi::xml_node newNode;
            if (events.empty()) {
                newNode = node.append_child("Maneuver");
            } else {
                newNode = node.insert_child_after("Maneuver", events.last().node);
            }

            newNode.append_attribute("name").set_value(name.toLocal8Bit().data());
            events.append(OscEvent(newNode));
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();

            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "Event") {
                    events.append(OscEvent(child));
                }
            }
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());
            for (auto event: events) {
                event.save();
            }
        }
    };

    class OscSequence {
    public:
        QString name;
        QVector<OscManeuver> maneuvers;
        pugi::xml_node node;

        OscSequence() {}

        OscSequence(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        bool removeManeuver(QString name) {
            for (int i = 0;i < maneuvers.size();i++) {
                if (maneuvers.at(i).name == name) {
                    node.remove_child(maneuvers.at(i).node);
                    maneuvers.remove(i);
                    return true;
                }
            }
            return false;
        }

        void addManeuver(QString name) {
            pugi::xml_node newNode;
            if (maneuvers.empty()) {
                newNode = node.append_child("Maneuver");
            } else {
                newNode = node.insert_child_after("Maneuver", maneuvers.last().node);
            }
            newNode.append_attribute("name").set_value(name.toLocal8Bit().data());
            maneuvers.append(OscManeuver(newNode));
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();
            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "Maneuver") {
                    maneuvers.append(OscManeuver(child));
                }
            }
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());
            for (auto maneuver: maneuvers) {
                maneuver.save();
            }
        }
    };

    class OscAct {
    public:
        QString name;
        QVector<OscSequence> sequences;
        pugi::xml_node node;

        OscAct() {}

        OscAct(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        bool removeSequence(QString name) {
            for (int i = 0;i < sequences.size();i++) {
                if (sequences.at(i).name == name) {
                    node.remove_child(sequences.at(i).node);
                    sequences.remove(i);
                    return true;
                }
            }
            return false;
        }

        void addSequence(QString name) {
            pugi::xml_node newNode;
            if (sequences.empty()) {
                newNode = node.prepend_child("Sequence");
            } else {
                newNode = node.insert_child_after("Sequence", sequences.last().node);
            }
            newNode.append_attribute("name").set_value(name.toLocal8Bit().data());
            sequences.append(OscSequence(newNode));
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.attribute("name").as_string();

            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "Sequence") {
                    sequences.append(OscSequence(child));
                }
            }
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());
            for (auto sequence: sequences) {
                sequence.save();
            }
        }
    };

    class OscStory {
    public:
        QString name;
        QVector<OscAct> acts;
        pugi::xml_node node;

        OscStory() {}

        OscStory(const pugi::xml_node xml_node) {
            load(xml_node);
        }

        void clear() {
            acts.clear();
        }

        void load(const pugi::xml_node xml_node) {
            node = xml_node;
            name = node.name();
            name = node.attribute("name").as_string();

            for (auto child = node.first_child(); child; child = child.next_sibling()) {
                if (QString(child.name()) == "Act") {
                    acts.append(OscAct(child));
                }
            }
        }

        void save() {
            node.attribute("name").set_value(name.toLocal8Bit().data());
            for (auto act: acts) {
                act.save();
            }
        }
    };

    OscStory mStory;
};

#endif // PAGESIMSCEN_H
