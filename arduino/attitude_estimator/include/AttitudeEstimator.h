#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <BasicLinearAlgebra.h>

class AttitudeEstimator
{
    public:
        AttitudeEstimator();

        void update(const BLA::Matrix<3, 1>& gyro, const BLA::Matrix<3, 1>& accel);
        BLA::Matrix<4, 1> getOrientation() const;

    private:
        BLA::Matrix<4, 1> orientation_;
};

#endif // ATTITUDE_ESTIMATOR_H