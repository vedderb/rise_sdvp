#ifndef SIMSCENTREE_H
#define SIMSCENTREE_H

#include <QObject>
#include <QString>
#include <QVector>
#include "pugixml.hpp"

typedef enum {
    OscActionUnknown = 0,
    OscActionLatLaneChAbs
} OscActionType;

class OscAction {
public:
    QString name;
    OscActionType type;
    pugi::xml_node node;

    QString latLaneChAbs_obj;
    bool latLaneChAbs_freespace;
    double latLaneChAbs_timegap;
    double latLaneChAbs_time;
    QString latLaneChAbs_shape;
    int latLaneChAbs_value;

    void load(const pugi::xml_node xml_node) {
        node = xml_node;
        name = node.attribute("name").as_string();
        type = OscActionUnknown;

        for (auto child = node.first_child(); child; child = child.next_sibling()) {
            if (QString(child.name()) == "PrivateAction") {
                for (auto child2 = child.first_child(); child2; child2 = child2.next_sibling()) {
                    if (QString(child2.name()) == "LateralAction") {
                        for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                            if (QString(child3.name()) == "LaneChangeAction") {
                                latLaneChAbs_obj = child3.attribute("object").as_string();
                                latLaneChAbs_freespace = QString(child3.attribute("freespace").as_string()).toLower() == "true";
                                latLaneChAbs_timegap = child3.attribute("timeGap").as_double();
                            }
                            for (auto child4 = child3.first_child(); child4; child4 = child4.next_sibling()) {
                                if (QString(child4.name()) == "LaneChangeActionDynamics") {
                                    latLaneChAbs_time = child4.attribute("time").as_double();
                                    latLaneChAbs_shape = child4.attribute("dynamicsShape").as_string("sinusoidal");
                                } else if (QString(child4.name()) == "LaneChangeTarget") {
                                    for (auto child5 = child4.first_child(); child5; child5 = child5.next_sibling()) {
                                        if (QString(child5.name()) == "AbsoluteTargetLane") {
                                            type = OscActionLatLaneChAbs;
                                            latLaneChAbs_value = child5.attribute("value").as_int();
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
        case OscActionLatLaneChAbs: {
            pugi::xml_node laneChange = node.child("PrivateAction").child("LateralAction").child("LaneChangeAction");
            laneChange.attribute("object").set_value(latLaneChAbs_obj.toLocal8Bit().data());
            laneChange.attribute("freespace").set_value(latLaneChAbs_freespace ? "true" : "false");
            laneChange.attribute("timeGap").set_value(latLaneChAbs_timegap);

            pugi::xml_node dynamics = laneChange.child("LaneChangeActionDynamics");
            dynamics.attribute("time").set_value(latLaneChAbs_time);
            dynamics.attribute("dynamicsShape").set_value(latLaneChAbs_shape.toLocal8Bit().data());

            laneChange.child("LaneChangeTarget").child("AbsoluteTargetLane").attribute("value").set_value(latLaneChAbs_value);
        } break;

        default:
            break;
        }
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
            if (QString(child.name()) == "ByEntityCondition") {
                for (auto child2 = child.first_child(); child2; child2 = child2.next_sibling()) {
                    if (QString(child2.name()) == "TriggeringEntities") {
                        for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                            if (QString(child3.name()) == "EntityRef") {
                                entityName = child3.attribute("name").as_string();
                            }
                        }
                    } else if (QString(child2.name()) == "EntityCondition") {
                        for (auto child3 = child2.first_child(); child3; child3 = child3.next_sibling()) {
                            if (QString(child3.name()) == "ReachPositionCondition") {
                                reachPosTolerance = child3.attribute("tolerance").as_double();
                                for (auto child4 = child3.first_child(); child4; child4 = child4.next_sibling()) {
                                    if (QString(child4.name()) == "Position") {
                                        for (auto child5 = child4.first_child(); child5; child5 = child5.next_sibling()) {
                                            if (QString(child5.name()) == "LanePosition") {
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
            node.child("ByEntityCondition").
                    child("TriggeringEntities").
                    child("EntityRef").attribute("name").
                    set_value(entityName.toLocal8Bit().data());
            node.child("ByEntityCondition").
                    child("EntityCondition").
                    child("ReachPositionCondition").
                    attribute("tolerance").
                    set_value(reachPosTolerance);

            pugi::xml_node lane = node.child("ByEntityCondition").
                    child("EntityCondition").
                    child("ReachPositionCondition").
                    child("Position").
                    child("LanePosition");

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
            } else if (QString(child.name()) == "StartTrigger") {
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
        node.attribute("name").set_value(name.toLocal8Bit().data());
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

    void addEvent(QString name, QString priority = "overwrite") {
        pugi::xml_node newNode;
        if (events.empty()) {
            newNode = node.append_child("Event");
        } else {
            newNode = node.insert_child_after("Event", events.last().node);
        }

        newNode.append_attribute("name").set_value(name.toLocal8Bit().data());
        newNode.append_attribute("priority").set_value(priority.toLocal8Bit().data());
        events.append(OscEvent(newNode));
    }

    void addEventLaneChangePos(QString name) {
        addEvent(name);
        pugi::xml_node eventNode = events.last().node;

        pugi::xml_node actionNode = eventNode.append_child("Action");
        actionNode.append_attribute("name").set_value(QString(name + "Action").toLocal8Bit().data());
        pugi::xml_node laneChangeNode = actionNode.append_child("PrivateAction").
                append_child("LateralAction").append_child("LaneChangeAction");
        laneChangeNode.append_attribute("object").set_value("Ego");
        laneChangeNode.append_attribute("freespace").set_value("true");
        laneChangeNode.append_attribute("timeGap").set_value(1.0);
        pugi::xml_node dynamicsNode = laneChangeNode.append_child("LaneChangeActionDynamics");
        dynamicsNode.append_attribute("time").set_value(2.0);
        dynamicsNode.append_attribute("dynamicsShape").set_value("sinusoidal");
        laneChangeNode.append_child("LaneChangeTarget").append_child("AbsoluteTargetLane").append_attribute("value").set_value(0);

        pugi::xml_node conditionNode = eventNode.append_child("StartTrigger").
                append_child("ConditionGroup").append_child("Condition");
        conditionNode.append_attribute("name").set_value(QString(name + "Condition").toLocal8Bit().data());
        conditionNode.append_attribute("delay").set_value(0.0);
        conditionNode.append_attribute("conditionEdge").set_value("rising");
        pugi::xml_node byEntNode = conditionNode.append_child("ByEntityCondition");
        pugi::xml_node triggerEntNode = byEntNode.append_child("TriggeringEntities");
        triggerEntNode.append_attribute("rule").set_value("any");
        triggerEntNode.append_child("EntityRef").append_attribute("name").set_value("Ego");
        pugi::xml_node reachPosNode = byEntNode.append_child("EntityCondition").append_child("ReachPositionCondition");
        reachPosNode.append_attribute("tolerance").set_value(1.0);
        pugi::xml_node laneNode = reachPosNode.append_child("Position").append_child("LanePosition");
        laneNode.append_attribute("roadId").set_value(0);
        laneNode.append_attribute("laneId").set_value(0);
        laneNode.append_attribute("s").set_value(0.0);

        events.removeLast();
        events.append(OscEvent(eventNode));
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
            newNode = node.prepend_child("ManeuverGroup");
        } else {
            newNode = node.insert_child_after("ManeuverGroup", sequences.last().node);
        }
        newNode.append_attribute("name").set_value(name.toLocal8Bit().data());
        sequences.append(OscSequence(newNode));
    }

    void load(const pugi::xml_node xml_node) {
        node = xml_node;
        name = node.attribute("name").as_string();

        for (auto child = node.first_child(); child; child = child.next_sibling()) {
            if (QString(child.name()) == "ManeuverGroup") {
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
