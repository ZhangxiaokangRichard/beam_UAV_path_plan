#pragma once

#include "ros_mqtt_bridge/message_types.h"

namespace ros_mqtt_bridge {

class Geodesy {
public:
    Geodesy(double origin_latitude_deg, double origin_longitude_deg,
            double origin_altitude_m);

    LocalPose toLocal(const GeodeticPosition& position) const;
    GeodeticPosition toGeodetic(double x, double y, double z) const;

private:
    double origin_latitude_rad_;
    double origin_longitude_rad_;
    double origin_altitude_m_;
    double origin_x_;
    double origin_y_;
    double origin_z_;

    void geodeticToEcef(const GeodeticPosition& position,
                        double& x, double& y, double& z) const;
    GeodeticPosition ecefToGeodetic(double x, double y, double z) const;
};

}  // namespace ros_mqtt_bridge