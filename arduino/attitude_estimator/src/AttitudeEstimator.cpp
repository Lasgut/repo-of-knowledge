#include "AttitudeEstimator.h"


AttitudeEstimator::AttitudeEstimator()
{
}

BLA::Matrix<4,1> 
AttitudeEstimator::getOrientation() const
{
    return orientation_;
}

void 
AttitudeEstimator::update(const BLA::Matrix<3,1> &gyro, const BLA::Matrix<3,1> &accel)
{
    // ax, ay, az in g's (or raw, but ratios are fine)
    float ax = accel(0);
    float ay = accel(1);
    float az = accel(2);

    // Calculate roll and pitch in radians
    auto roll  = atan2(ay, az);
    auto pitch = atan2(-ax, sqrt(ay*ay + az*az));

    orientation_(0) = roll;
    orientation_(1) = pitch;
    orientation_(2) = 0;
}
