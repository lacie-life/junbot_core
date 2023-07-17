#ifndef QROBOTULTIS_H
#define QROBOTULTIS_H

#include <QSizeF>
#include <math.h>

struct QRobotPose {
    double x;
    double y;
    double z;
    double theta;
};

struct QRobotSpeed {
    double vx;
    double vy;
    double w;
};

inline double deg2rad(double x) {
    return M_PI * x / 180.0;
}

inline double rad2deg(double x) {
    return 180.0 * x / M_PI;
}

inline double getAngle(float x1, float y1, float x2, float y2) {
    float angle_temp;
    float xx, yy;
    xx = x2 - x1;
    yy = y2 - y1;
    if (xx == 0.0)
        angle_temp = M_PI / 2.0;
    else
        angle_temp = atan(fabs(yy / xx));
    if ((xx < 0.0) && (yy >= 0.0))
        angle_temp = M_PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
        angle_temp = M_PI + angle_temp;
    else if ((xx >= 0.0) && (yy < 0.0))
        angle_temp = M_PI * 2.0 - angle_temp;
    return (angle_temp);
}

#endif // QROBOTULTIS_H
