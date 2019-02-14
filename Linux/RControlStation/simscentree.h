#ifndef SIMSCENTREE_H
#define SIMSCENTREE_H

#include <QObject>
#include <QString>
#include "pugixml.hpp"

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
        type = OscCondUnknown;

        for (auto child = node.first_child(); child; child = child.next_sibling()) {
            if (QString(child.name()) == "ByEntity") {
                for (auto child2 = child.first_child(); child2; child2 = child2.next_sibling()) {
                    if (QString(child2.name()) == "TriggeringEntities") {
                        for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                            if (QString(child3.name()) == "Entity") {
                                entityName = child3.attribute("name").as_string();
                            }
                        }
                    } else if (QString(child2.name()) == "EntityCondition") {
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

Q_DECLARE_METATYPE(OscStory*)
Q_DECLARE_METATYPE(OscAct*)
Q_DECLARE_METATYPE(OscSequence*)
Q_DECLARE_METATYPE(OscManeuver*)
Q_DECLARE_METATYPE(OscEvent*)
Q_DECLARE_METATYPE(OscAction*)
Q_DECLARE_METATYPE(OscCondition*)

#endif // SIMSCENTREE_H
