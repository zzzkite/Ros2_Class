#pragma once

#include <math.h>

#include <QObject>

class DummyClass : public QObject
{
    Q_OBJECT
};

inline double deg2rad(double x)
{
    return M_PI * x / 180.0;
}

inline double rad2deg(double x)
{
    return 180.0 * x / M_PI;
}

class RobotPose
{
  public:
    double x{0};
    double y{0};
    double theta{0};
};

class RobotSpeed
{
  public:
    double vx{0};
    double vy{0};
    double w{0};
};

class RobotState : public RobotPose, public RobotSpeed
{
};