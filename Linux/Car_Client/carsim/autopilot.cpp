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

#include "autopilot.h"
#include "utility.h"
#include <cmath>
#include <QDebug>

namespace
{
float utils_point_distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

int utils_truncate_number_abs(float *number, float max) {
    int did_trunc = 0;

    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < -max) {
        *number = -max;
        did_trunc = 1;
    }

    return did_trunc;
}

float utils_rp_distance(const ROUTE_POINT *p1, const ROUTE_POINT *p2) {
    float dx = p2->px - p1->px;
    float dy = p2->py - p1->py;
    return sqrtf(dx * dx + dy * dy);
}

bool utils_time_before(int32_t t1, int32_t t2) {
    if (t1 < t2 && (t2 - t1) < ((24 * 60 * 60 * 1000) / 2)) {
        return true;
    } else {
        return false;
    }
}

int utils_circle_line_int(float cx, float cy, float rad,
        const ROUTE_POINT *point1, const ROUTE_POINT *point2,
        ROUTE_POINT *int1, ROUTE_POINT *int2) {
    float dx, dy, a, b, c, det, t, x, y;

    const float p1x = point1->px;
    const float p1y = point1->py;
    const float p2x = point2->px;
    const float p2y = point2->py;

    float maxx = p1x;
    float minx = p2x;
    float maxy = p1y;
    float miny = p2y;

    if (maxx < minx) {
        maxx = p2x;
        minx = p1x;
    }

    if (maxy < miny) {
        maxy = p2y;
        miny = p1y;
    }

    dx = p2x - p1x;
    dy = p2y - p1y;

    a = dx * dx + dy * dy;
    b = 2 * (dx * (p1x - cx) + dy * (p1y - cy));
    c = (p1x - cx) * (p1x - cx) + (p1y - cy) * (p1y - cy) - rad * rad;

    det = b * b - 4 * a * c;

    int ints = 0;
    if ((a <= 1e-6) || (det < 0.0)) {
        // No real solutions.
    } else if (det == 0) {
        // One solution.
        t = -b / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            int1->px = x;
            int1->py = y;
            ints++;
        }
    } else {
        // Two solutions.
        t = (-b + sqrtf(det)) / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            int1->px = x;
            int1->py = y;
            ints++;
        }

        t = (-b - sqrtf(det)) / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            if (ints) {
                int2->px = x;
                int2->py = y;
            } else {
                int1->px = x;
                int1->py = y;
            }

            ints++;
        }
    }

    return ints;
}

void utils_closest_point_line(const ROUTE_POINT *point1, const ROUTE_POINT *point2,
        float px, float py, ROUTE_POINT *res) {
    const float p1x = point1->px;
    const float p1y = point1->py;
    const float p2x = point2->px;
    const float p2y = point2->py;

    const float d1x = px - p1x;
    const float d1y = py - p1y;
    const float dx = p2x - p1x;
    const float dy = p2y - p1y;

    const float ab2 = dx * dx + dy * dy;
    const float ap_ab = d1x * dx + d1y * dy;
    float t = ap_ab / ab2;

    if (t < 0.0) {
        t = 0.0;
    } else if (t > 1.0) {
        t = 1.0;
    }

    res->px = p1x + dx * t;
    res->py = p1y + dy * t;
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
}

Autopilot::Autopilot(QObject *parent) : QObject(parent)
{
    mPx = 0.0;
    mPy = 0.0;
    mSpeed = 0.0;

    mAxisDistance = 0.475;
    mRepeatRoutes = true;
    mSpeedMax = 100.0;
    mBaseRad = 0.8;
    mRadTime = 0.8;
    mTimeAddMs = 1000 * 60;
    mModeTime = 0;

    m_route = new ROUTE_POINT[ap_route_size];

    memset(m_route, 0, sizeof(ROUTE_POINT) * ap_route_size);
    m_is_active = false;
    m_point_now = 0;
    m_point_last = 0;
    m_has_prev_point = false;
    m_override_speed = 0.0;
    m_is_speed_override = false;
    memset(&m_rp_now, 0, sizeof(ROUTE_POINT));
    m_rad_now = -1.0;
    memset(&m_point_rx_prev, 0, sizeof(ROUTE_POINT));
    m_point_rx_prev_set = false;
    m_start_time = 0;
    m_sync_rx = false;
    m_print_closest_point = false;
    m_en_dynamic_rad = true;
    m_en_angle_dist_comp = true;
    m_route_look_ahead = 8;
    m_route_left = 0;

    mTimer = new QTimer(this);
    mTimer->start(10);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

Autopilot::~Autopilot()
{
    delete[] m_route;
}

bool Autopilot::autopilot_add_point(ROUTE_POINT *p, bool first)
{
    return add_point(p, first);
}

void Autopilot::autopilot_remove_last_point()
{
    if (m_point_last != m_point_now) {
        m_point_last--;
        if (m_point_last < 0) {
            m_point_last = ap_route_size - 1;
        }
    }
}

void Autopilot::autopilot_clear_route()
{
    clear_route();
}

bool Autopilot::autopilot_replace_route(ROUTE_POINT *p)
{
    bool ret = false;

    if (!m_is_active) {
        clear_route();
        add_point(p, true);
        ret = true;
    } else {
        bool time_mode = p->time > 0;

        while (m_point_last != m_point_now) {
            m_point_last--;
            if (m_point_last < 0) {
                m_point_last = ap_route_size - 1;
            }

            // If we use time stamps (times are > 0), only overwrite the newer
            // part of the route.
            if (time_mode && p->time >= m_route[m_point_last].time) {
                break;
            }
        }

        m_has_prev_point = m_point_last != m_point_now;

        // In time mode, only add the point if its timestamp was ahead of the point
        // we currently follow.
        if (!time_mode || m_point_last != m_point_now || p->time >= m_route[m_point_last].time) {
            add_point(p, true);
            ret = true;
        }
    }

    return ret;
}

void Autopilot::autopilot_sync_point(int32_t point, int32_t time, int32_t min_time_diff)
{
    int start = m_point_now + 1;
    if (start >= ap_route_size) {
        start = 0;
    }

    if (start == m_point_last) {
        return;
    }

    // Car center
    const float car_cx = mPx;
    const float car_cy = mPy;
    ROUTE_POINT car_pos;
    car_pos.px = car_cx;
    car_pos.py = car_cy;

    int point_i = start;
    int point_prev = 0;
    float dist_tot = 0.0;

    // Calculate remaining length
    for (;;) {
        if (point_i == start) {
            dist_tot += utils_rp_distance(&car_pos, &m_route[point_i]);
        } else {
            dist_tot += utils_rp_distance(&m_route[point_prev], &m_route[point_i]);
        }

        if (point_i == (m_point_last - 1) || point_i == point) {
            break;
        }

        point_prev = point_i;
        point_i++;
        if (point_i >= ap_route_size) {
            point_i = 0;
        }
    }

    float speed = dist_tot / ((float)time / 1000.0);
    utils_truncate_number_abs(&speed, mSpeedMax);

    if (time < min_time_diff || dist_tot < mBaseRad) {
        //m_sync_rx = false;
        return;
    }

    point_i = m_point_now;
    while (point_i <= point && point_i != m_point_last) {
        m_route[point_i].speed = speed;

        point_i++;
        if (point_i >= ap_route_size) {
            point_i = 0;
        }
    }

    m_sync_rx = true;
}

void Autopilot::autopilot_set_active(bool active)
{
    if (active && !m_is_active) {
        m_start_time = utility::getTimeUtcToday();
        //m_sync_rx = false;
    }

    m_is_active = active;
}

bool Autopilot::autopilot_is_active()
{
    return m_is_active;
}

int Autopilot::autopilot_get_route_len()
{
    return m_point_last;
}

int Autopilot::autopilot_get_point_now()
{
    return m_point_now;
}

int Autopilot::autopilot_get_route_left()
{
    return m_route_left;
}

ROUTE_POINT Autopilot::autopilot_get_route_point(int ind)
{
    ROUTE_POINT res;
    memset(&res, 0, sizeof(ROUTE_POINT));

    if (ind < m_point_last) {
        res = m_route[ind];
    }

    return res;
}

void Autopilot::autopilot_set_speed_override(bool is_override, float speed)
{
    m_is_speed_override = is_override;
    m_override_speed = speed;
}

void Autopilot::autopilot_set_motor_speed(float speed)
{
    emit setSpeed(speed);
}

float Autopilot::autopilot_get_steering_scale()
{
    const float div = 1.0 + fabsf(mSpeed) * 0.05;
    return 1.0 / (div * div);
}

float Autopilot::autopilot_get_rad_now()
{
    return m_rad_now;
}

void Autopilot::autopilot_get_goal_now(ROUTE_POINT *rp)
{
    *rp = m_rp_now;
}

void Autopilot::setAxisDistance(double axisDistance)
{
    mAxisDistance = axisDistance;
}

double Autopilot::axisDistance() const
{
    return mAxisDistance;
}

bool Autopilot::repeatRoutes() const
{
    return mRepeatRoutes;
}

void Autopilot::setRepeatRoutes(bool repeatRoutes)
{
    mRepeatRoutes = repeatRoutes;
}

double Autopilot::speedMax() const
{
    return mSpeedMax;
}

void Autopilot::setSpeedMax(double speedMax)
{
    mSpeedMax = speedMax;
}

void Autopilot::updatePositionSpeed(double px, double py, double yaw, double speed)
{
    mPx = px;
    mPy = py;
    mYaw = yaw;
    mSpeed = speed;
}

double Autopilot::baseRad() const
{
    return mBaseRad;
}

void Autopilot::setBaseRad(double baseRad)
{
    mBaseRad = baseRad;
}

double Autopilot::radTime() const
{
    return mRadTime;
}

void Autopilot::setRadTime(double radTime)
{
    mRadTime = radTime;
}

int Autopilot::timeAddMs() const
{
    return mTimeAddMs;
}

void Autopilot::setTimeAddMs(int timeAddMs)
{
    mTimeAddMs = timeAddMs;
}

int Autopilot::modeTime() const
{
    return mModeTime;
}

void Autopilot::setModeTime(int modeTime)
{
    mModeTime = modeTime;
}

void Autopilot::timerSlot()
{
    bool route_end = false;

    if (!m_is_active) {
        m_rad_now = -1.0;
        return;
    }

    // the length of the route that is left
    int len = m_point_last;

    // This means that the route has wrapped around
    // (should only happen when ap_repeat_routes == false)
    if (m_point_now > m_point_last) {
        len = ap_route_size + m_point_last - m_point_now;
    }

    m_route_left = len - m_point_now;
    if (m_route_left < 0) {
        m_route_left += ap_route_size;
    }

    // Time of today according to our clock
    int ms_today = utility::getTimeUtcToday();

    if (len >= 2) {
        // Car center
        const float car_cx = mPx;
        const float car_cy = mPy;
        ROUTE_POINT car_pos;
        car_pos.px = car_cx;
        car_pos.py = car_cy;

        // Look m_route_look_ahead points ahead, or less than that if the route is shorter
        int add = m_route_look_ahead;
        if (add > len) {
            add = len;
        }

        int start = m_point_now;
        int end = m_point_now + add;

        // Speed-dependent radius
        if (m_en_dynamic_rad) {
            m_rad_now = fabsf(mRadTime * mSpeed);
            if (m_rad_now < mBaseRad) {
                m_rad_now = mBaseRad;
            }
        } else {
            m_rad_now = mBaseRad;
        }

        ROUTE_POINT rp_now; // The point we should follow now.
        int circle_intersections = 0;
        bool last_point_reached = false;

        // Last point in route
        int last_point_ind = m_point_last - 1;
        if (last_point_ind < 0) {
            last_point_ind += ap_route_size;
        }

        ROUTE_POINT *rp_last = &m_route[last_point_ind]; // Last point on route
        ROUTE_POINT *rp_ls1 = &m_route[0]; // First point on goal line segment
        ROUTE_POINT *rp_ls2 = &m_route[1]; // Second point on goal line segment

        ROUTE_POINT *closest1_speed = &m_route[0];
        ROUTE_POINT *closest2_speed = &m_route[1];

        for (int i = start;i < end;i++) {
            int ind = i; // First point index for this iteration
            int indn = i + 1; // Next point index for this iteration

            // Wrap around
            if (ind >= m_point_last) {
                if (m_point_now <= m_point_last) {
                    ind -= m_point_last;
                } else {
                    if (ind >= ap_route_size) {
                        ind -= ap_route_size;
                    }
                }
            }

            // Wrap around
            if (indn >= m_point_last) {
                if (m_point_now <= m_point_last) {
                    indn -= m_point_last;
                } else {
                    if (indn >= ap_route_size) {
                        indn -= ap_route_size;
                    }
                }
            }

            // Check for circle intersection. If there are many intersections
            // found in this loop, the last one will be used.
            ROUTE_POINT int1, int2;
            ROUTE_POINT *p1, *p2;
            p1 = &m_route[ind];
            p2 = &m_route[indn];

            // If the next point has a time before the current point and repeat route is
            // active we have completed a full route. Increase its time by the repetition time.
            if (mRepeatRoutes && utils_time_before(p2->time, p1->time)) {
                p2->time += mTimeAddMs;
                if (p2->time > ms_per_day) {
                    p2->time -= ms_per_day;
                }
            }

            int res = utils_circle_line_int(car_cx, car_cy, m_rad_now, p1, p2, &int1, &int2);

            if (res) {
                closest1_speed = p1;
                closest2_speed = p2;
            }

            // One intersection. Use it.
            if (res == 1) {
                circle_intersections++;
                rp_now = int1;
            }

            // Two intersections. Use the point with the most "progress" on the route.
            if (res == 2) {
                circle_intersections += 2;

                if (utils_rp_distance(&int1, p2) < utils_rp_distance(&int2, p2)) {
                    rp_now = int1;
                } else {
                    rp_now = int2;
                }
            }

            if (res > 0) {
                rp_ls1 = &m_route[ind];
                rp_ls2 = &m_route[indn];
            }

            // If we aren't repeating routes and there is an intersection on the last
            // line segment, go straight to the last point.
            if (!mRepeatRoutes) {
                if (indn == last_point_ind && circle_intersections > 0) {
                    if (res > 0) {
                        last_point_reached = true;
                    }
                    break;
                }
            }
        }

        // Look for closest points
        ROUTE_POINT closest; // Closest point on route to car
        ROUTE_POINT *closest1 = &m_route[0]; // Start of closest line segment
        ROUTE_POINT *closest2 = &m_route[1]; // End of closest line segment
        int closest1_ind = 0; // Index of the first closest point

        {
            bool closest_set = false;

            for (int i = start;i < end;i++) {
                int ind = i; // First point index for this iteration
                int indn = i + 1; // Next point index for this iteration

                // Wrap around
                if (ind >= m_point_last) {
                    if (m_point_now <= m_point_last) {
                        ind -= m_point_last;
                    } else {
                        if (ind >= ap_route_size) {
                            ind -= ap_route_size;
                        }
                    }
                }

                // Wrap around
                if (indn >= m_point_last) {
                    if (m_point_now <= m_point_last) {
                        indn -= m_point_last;
                    } else {
                        if (indn >= ap_route_size) {
                            indn -= ap_route_size;
                        }
                    }
                }

                ROUTE_POINT tmp;
                ROUTE_POINT *p1, *p2;
                p1 = &m_route[ind];
                p2 = &m_route[indn];
                utils_closest_point_line(p1, p2, car_cx, car_cy, &tmp);

                if (!closest_set || utils_rp_distance(&tmp, &car_pos) < utils_rp_distance(&closest, &car_pos)) {
                    closest_set = true;
                    closest = tmp;
                    closest1 = p1;
                    closest2 = p2;
                    closest1_ind = ind;
                }

                // Do not look past the last point if we aren't repeating routes.
                if (!mRepeatRoutes) {
                    if (indn == last_point_ind) {
                        break;
                    }
                }
            }
        }

        if (circle_intersections == 0) {
            closest1_speed = closest1;
            closest2_speed = closest2;
        }

        if (last_point_reached) {
            rp_now = *rp_last;
        } else {
            // Use the closest point on the considered route if no
            // circle intersection is found.
            if (circle_intersections == 0) {
                rp_now = closest;
                rp_ls1 = closest1;
                rp_ls2 = closest2;
            }
        }

        // Check if the end of route is reached
        if (!mRepeatRoutes && m_route_left < 3 &&
                utils_rp_distance(&m_route[last_point_ind], &car_pos) < m_rad_now) {
            route_end = true;
        }

        m_point_now = closest1_ind;
        m_rp_now = rp_now;

        if (!route_end) {
            float distance = 0.0;
            float steering_angle = 0.0;
            float circle_radius = 1000.0;

            steering_angle_to_point(mPx, mPy, -mYaw * M_PI / 180.0, rp_now.px,
                                    rp_now.py, &steering_angle, &distance, &circle_radius);

            float speed = 0.0;

            if (mModeTime && !m_sync_rx) {
                if (ms_today >= 0) {
                    // Calculate speed such that the route points are reached at their
                    // specified time. Notice that the direct distance between the car
                    // and the points is used and not the arc that the car drives. This
                    // should still work well enough.

                    int32_t dist_prev = (int32_t)(utils_rp_distance(&rp_now, rp_ls1) * 1000.0);
                    int32_t dist_tot = (int32_t)(utils_rp_distance(&rp_now, rp_ls1) * 1000.0);
                    dist_tot += (int32_t)(utils_rp_distance(&rp_now, rp_ls2) * 1000.0);
                    int32_t time = utils_map_int(dist_prev, 0, dist_tot, rp_ls1->time, rp_ls2->time);
                    float dist_car = utils_rp_distance(&car_pos, &rp_now);

                    int32_t t_diff = time - ms_today;

                    if (mModeTime == 2) {
                        t_diff += m_start_time;
                    }

                    if (t_diff < 0) {
                        t_diff += 24 * 60 * 60 * 1000;
                    }

                    if (t_diff > 0) {
                        speed = dist_car / ((float)t_diff / 1000.0);
                    } else {
                        speed = 0.0;
                    }
                } else {
                    speed = 0.0;
                }
            } else {
                // Calculate the speed based on the average speed between the two closest points
                const float dist_prev = utils_rp_distance(&rp_now, closest1_speed);
                const float dist_tot = utils_rp_distance(&rp_now, closest1_speed) + utils_rp_distance(&rp_now, closest2_speed);
                speed = utils_map(dist_prev, 0.0, dist_tot, closest1_speed->speed, closest2_speed->speed);
            }

            if (m_is_speed_override) {
                speed = m_override_speed;
            }

            utils_truncate_number_abs(&speed, mSpeedMax);

            emit setSteeringTurnRad(circle_radius);
            autopilot_set_motor_speed(speed);
        }
    } else {
        route_end = true;
    }

    if (route_end) {
        emit setCurrentBrake(-50.0);
        m_rad_now = -1.0;
    }
}

void Autopilot::steering_angle_to_point(float current_x,
                                        float current_y,
                                        float current_angle,
                                        float goal_x,
                                        float goal_y,
                                        float *steering_angle,
                                        float *distance,
                                        float *circle_radius)
{
    const float D = utils_point_distance(goal_x, goal_y, current_x, current_y);
    *distance = D;
    const float gamma = current_angle - atan2f((goal_y-current_y), (goal_x-current_x));
    const float dx = D * cosf(gamma);
    const float dy = D * sinf(gamma);

    if (fabsf(dy) <= 0.000001) {
        *steering_angle = 0.0;
        *circle_radius = 9000.0;
        return;
    }

    float R = -(dx * dx + dy * dy) / (2.0 * dy);

    /*
         * Add correction if the arc is much longer than the total distance.
         * TODO: Find a good model.
         */
    float angle_correction = 1.0 + (m_en_angle_dist_comp ? D * 0.2 : 0.0);
    if (angle_correction > 5.0) {
        angle_correction = 5.0;
    }

    R /= angle_correction;

    *circle_radius = R;
    *steering_angle = atanf(mAxisDistance / R);
}

bool Autopilot::add_point(ROUTE_POINT *p, bool first)
{
    if (first && m_point_rx_prev_set &&
            utils_point_distance(m_point_rx_prev.px, m_point_rx_prev.py, p->px, p->py) < 1e-4) {
        return false;
    }

    if (first) {
        m_point_rx_prev = *p;
        m_point_rx_prev_set = true;
    }

    m_route[m_point_last++] = *p;

    if (m_point_last >= ap_route_size) {
        m_point_last = 0;
    }

    // Make sure that there always is a valid point when looking backwards in the route
    if (!m_has_prev_point) {
        int p_last = m_point_now - 1;
        if (p_last < 0) {
            p_last += ap_route_size;
        }

        m_route[p_last] = *p;
        m_has_prev_point = true;
    }

    // When repeating routes, the previous point for the first
    // point is the end point of the current route.
    if (mRepeatRoutes) {
        m_route[ap_route_size - 1] = *p;
    }

    return true;
}

void Autopilot::clear_route()
{
    m_is_active = false;
    m_has_prev_point = false;
    m_point_now = 0;
    m_point_last = 0;
    m_point_rx_prev_set = false;
    m_start_time = utility::getTimeUtcToday();
    m_sync_rx = false;
    memset(&m_rp_now, 0, sizeof(ROUTE_POINT));
    memset(&m_point_rx_prev, 0, sizeof(ROUTE_POINT));
}
