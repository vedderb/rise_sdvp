#pragma once

#include <functional>
#include <atomic>
#include <thread>
#include "../rcontrolstationcomm.h"

/**
 * @brief The RCSLocationProvider class
 * This class provides periodic reports on the RCS location of the system.
 */
class RCSLocationProvider {
public:
    typedef std::function<void(double lat, double lon)> location_callback_t;

    RCSLocationProvider();

    ~RCSLocationProvider();

    void request_location_updates(QString host, int port, int vehicleID, location_callback_t callback);
    bool is_running() { return !should_exit_; };

private:
    void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z);
    void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height);
    void createEnuMatrix(double lat, double lon, double *enuMat);
    void enuToLlh(const double *iLlh, const double *xyz, double *llh);

    void start();
    void stop();
    void get_rcs_locations();

    std::thread* thread_{nullptr};
    std::atomic<bool> should_exit_{false};

    QString host_;
    int port_;
    int vehicleID_ = -1;
    location_callback_t location_callback_ = nullptr;
    double latitude_deg_ = 57.76956;
    double longitude_deg_ = 12.82094;
    size_t count_ = 0u;

    static const size_t MAX_LOCATIONS;
    static const double LATITUDE_DEG_PER_METER;
    static const double LONGITUDE_DEG_PER_METER;
};
