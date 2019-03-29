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

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <QObject>
#include <QTimer>

typedef struct {
    float px;
    float py;
    float pz;
    float speed;
    int32_t time;
} ROUTE_POINT;

class Autopilot : public QObject
{
    Q_OBJECT
public:
    explicit Autopilot(QObject *parent = nullptr);
    ~Autopilot();

    // Same C functions as in the firmware
    bool autopilot_add_point(ROUTE_POINT *p, bool first);
    void autopilot_remove_last_point(void);
    void autopilot_clear_route(void);
    bool autopilot_replace_route(ROUTE_POINT *p);
    void autopilot_sync_point(int32_t point, int32_t time, int32_t min_time_diff);
    void autopilot_set_active(bool active);
    bool autopilot_is_active(void);
    int autopilot_get_route_len(void);
    int autopilot_get_point_now(void);
    int autopilot_get_route_left(void);
    ROUTE_POINT autopilot_get_route_point(int ind);
    void autopilot_set_speed_override(bool is_override, float speed);
    void autopilot_set_motor_speed(float speed);
    float autopilot_get_steering_scale(void);
    float autopilot_get_rad_now(void);
    void autopilot_get_goal_now(ROUTE_POINT *rp);

    double axisDistance() const;
    void setAxisDistance(double axisDistance);

    bool repeatRoutes() const;
    void setRepeatRoutes(bool repeatRoutes);

    double speedMax() const;
    void setSpeedMax(double speedMax);

    double baseRad() const;
    void setBaseRad(double baseRad);

    int timeAddMs() const;
    void setTimeAddMs(int timeAddMs);

    int modeTime() const;
    void setModeTime(int modeTime);

signals:
    void setSpeed(double speed);
    void setCurrentBrake(double current);
    void setSteeringTurnRad(double turnRad);

public slots:
    void updatePositionSpeed(double px, double py, double yaw, double speed);

private slots:
    void timerSlot();

private:
    static const int ap_route_size = 500;
    static const int ms_per_day = (24 * 60 * 60 * 1000);

    QTimer *mTimer;
    double mPx;
    double mPy;
    double mSpeed;
    double mYaw;

    // Configuration
    double mAxisDistance;
    bool mRepeatRoutes;
    double mSpeedMax;
    double mBaseRad;
    int mTimeAddMs;
    int mModeTime;

    ROUTE_POINT *m_route;
    bool m_is_active;
    int m_point_last; // The last point on the route
    int m_point_now; // The first point in the currently considered part of the route
    bool m_has_prev_point;
    float m_override_speed;
    bool m_is_speed_override;
    ROUTE_POINT m_rp_now; // The point in space we are following now
    float m_rad_now;
    ROUTE_POINT m_point_rx_prev;
    bool m_point_rx_prev_set;
    int32_t m_start_time;
    bool m_sync_rx;
    int m_print_closest_point;
    bool m_en_dynamic_rad;
    bool m_en_angle_dist_comp;
    int m_route_look_ahead;
    int m_route_left;

    void steering_angle_to_point(
            float current_x,
            float current_y,
            float current_angle,
            float goal_x,
            float goal_y,
            float *steering_angle,
            float *distance,
            float *circle_radius);
    bool add_point(ROUTE_POINT *p, bool first);
    void clear_route(void);
};

#endif // AUTOPILOT_H
