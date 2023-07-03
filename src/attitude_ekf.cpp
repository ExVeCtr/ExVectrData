#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrMath.hpp"

#include "ExVectrData/value_covariance.hpp"

#include "ExVectrData/attitude_ekf.hpp"

namespace VCTR
{

    namespace Data
    {

        AttitudeEKF::AttitudeEKF()
        {
            setState(ValueCov<float, 7>{0, 1000});
            q_ = 1;
            stateTimestamp_ = VCTR::Core::NOW();
        }

        void AttitudeEKF::predict(int64_t time)
        {

            float dTime = static_cast<float>(time - stateTimestamp_) / VCTR::Core::NANOSECONDS;
            stateTimestamp_ = time;

            VCTR::Math::Quat<float> quat = x_.block<4, 1>(3, 0);

            quat = VCTR::Math::Quat<float>::fromRotVec(x_.block<3, 1>(0, 0)) * quat;

            x_.block(quat, 3, 0);

        }

        void AttitudeEKF::updateBodyAngVel(const VCTR::Math::Vector<float, 3> &input, const VCTR::Math::Matrix<float, 3, 3> &inputCov)
        {

            VCTR::Math::Quat<float> quat = x_.block<4, 1>(3, 0);

            // Transform from body to reference frame.
            auto angVel = quat.conjugate().rotate(input);
            auto angVelCov = quat.conjugate().rotate(inputCov);

            x_.block(angVel, 0, 0);

        }

        void AttitudeEKF::updateBodyDirection(const VCTR::Math::Vector<float, 3> &input, const VCTR::Math::Matrix<float, 3, 3> &inputCov, VCTR::Math::Vector<float, 3> &reference)
        {
            
            VCTR::Math::Quat<float> quat = x_.block<4, 1>(3, 0);

            // Transform from body to reference frame.
            auto inWrld = quat.rotate(input);
            auto inWrldCov = quat.rotate(inputCov);

            auto rot = inWrld.cross(reference);

            x_.block(VCTR::Math::Quat<float>(rot.normalize(), -rot.magnitude() * 0.1) * quat, 3, 0);

        }

        void AttitudeEKF::setProcessNoise(const VCTR::Math::Matrix<float, 7, 7> &noise)
        {
            q_ = noise;
        }

        const VCTR::Math::Vector<float, 7> &AttitudeEKF::getState()
        {
            return x_;
        }

        const VCTR::Math::Matrix<float, 7, 7> &AttitudeEKF::getCovariance()
        {
            return p_;
        }

        void AttitudeEKF::setState(const ValueCov<float, 7> &state)
        {
            x_ = state.val;
            p_ = state.cov;
        }

    }
}