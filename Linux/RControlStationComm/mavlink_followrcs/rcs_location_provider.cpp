
#include "rcs_location_provider.h"
#include <chrono> // for seconds()
#include <thread> // for sleep_for()
#include <cmath>
#include <iostream>

using std::this_thread::sleep_for;
using std::chrono::milliseconds;

#define FE_WGS84        (1.0/298.257223563) // earth flattening (WGS84)
#define RE_WGS84        6378137.0           // earth semimajor axis (WGS84) (m)

void RCSLocationProvider::llhToXyz(double lat, double lon, double height, double *x, double *y, double *z)
{
    double sinp = sin(lat * M_PI / 180.0);
    double cosp = cos(lat * M_PI / 180.0);
    double sinl = sin(lon * M_PI / 180.0);
    double cosl = cos(lon * M_PI / 180.0);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    *x = (v + height) * cosp * cosl;
    *y = (v + height) * cosp * sinl;
    *z = (v * (1.0 - e2) + height) * sinp;
}

void RCSLocationProvider::xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height)
{
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double r2 = x * x + y * y;
    double za = z;
    double zk = 0.0;
    double sinp = 0.0;
    double v = RE_WGS84;

    while (fabs(za - zk) >= 1E-6) {
        zk = za;
        sinp = za / sqrt(r2 + za * za);
        v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
        za = z + v * e2 * sinp;
    }

    *lat = (r2 > 1E-12 ? atan(za / sqrt(r2)) : (z > 0.0 ? M_PI / 2.0 : -M_PI / 2.0)) * 180.0 / M_PI;
    *lon = (r2 > 1E-12 ? atan2(y, x) : 0.0) * 180.0 / M_PI;
    *height = sqrt(r2 + za * za) - v;
}

void RCSLocationProvider::createEnuMatrix(double lat, double lon, double *enuMat)
{
    double so = sin(lon * M_PI / 180.0);
    double co = cos(lon * M_PI / 180.0);
    double sa = sin(lat * M_PI / 180.0);
    double ca = cos(lat * M_PI / 180.0);

    // ENU
    enuMat[0] = -so;
    enuMat[1] = co;
    enuMat[2] = 0.0;

    enuMat[3] = -sa * co;
    enuMat[4] = -sa * so;
    enuMat[5] = ca;

    enuMat[6] = ca * co;
    enuMat[7] = ca * so;
    enuMat[8] = sa;
}

void RCSLocationProvider::enuToLlh(const double *iLlh, const double *xyz, double *llh)
{
    double ix, iy, iz;
    llhToXyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

    double enuMat[9];
    createEnuMatrix(iLlh[0], iLlh[1], enuMat);

    double x = enuMat[0] * xyz[0] + enuMat[3] * xyz[1] + enuMat[6] * xyz[2] + ix;
    double y = enuMat[1] * xyz[0] + enuMat[4] * xyz[1] + enuMat[7] * xyz[2] + iy;
    double z = enuMat[2] * xyz[0] + enuMat[5] * xyz[1] + enuMat[8] * xyz[2] + iz;

    xyzToLlh(x, y, z, &llh[0], &llh[1], &llh[2]);
}

RCSLocationProvider::RCSLocationProvider() {}

RCSLocationProvider::~RCSLocationProvider()
{
    stop();
}

void RCSLocationProvider::request_location_updates(QString host, int port, int vehicleID, location_callback_t callback)
{
    host_ = host;
    port_ = port;
    vehicleID_ = vehicleID;
    location_callback_ = callback;
    stop();
    start();
}

void RCSLocationProvider::start()
{
    should_exit_ = false;
    thread_ = new std::thread(&RCSLocationProvider::get_rcs_locations, this);
}

void RCSLocationProvider::stop()
{
    should_exit_ = true;

    if (thread_) {
        thread_->join();
        delete thread_;
        thread_ = nullptr;
    }
}

void RCSLocationProvider::get_rcs_locations()
{
    RControlStationComm rcscomm;
    rcscomm.connectTcp(host_, port_);

    double iLlh[3];
    should_exit_ = !rcscomm.getEnuRef(vehicleID_, true, iLlh);

    while (!should_exit_) {
        if (rcscomm.hasError())
            should_exit_ = true;

        CAR_STATE vehicleState;
        should_exit_ = !rcscomm.getState(vehicleID_, &vehicleState);

        double xyz[3] {vehicleState.px, vehicleState.py, 100.0};
        double llh[3];
        std::cout.precision(8);
        std::cout << std::fixed << "ENU: " << iLlh[0] << ", " << iLlh[1] << ", " << iLlh[2] << " - xyz: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << std::endl;
        enuToLlh(iLlh, xyz, llh);

        location_callback_(llh[0], llh[1]);

        sleep_for(milliseconds(100));
    }
}

const double RCSLocationProvider::LATITUDE_DEG_PER_METER = 0.000009044;
const double RCSLocationProvider::LONGITUDE_DEG_PER_METER = 0.000008985;
