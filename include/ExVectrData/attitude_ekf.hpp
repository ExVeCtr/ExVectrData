#ifndef EXVECTRDATA_ATTITUDEEKF_H
#define EXVECTRDATA_ATTITUDEEKF_H

#include "ExVectrMath.hpp"

#include "value_covariance.hpp"

namespace VCTR
{

    namespace Data
    {

        /**
         * @brief A class implementing a quaternion based EKF to estimate an rotation from world to body or aka attitude.
         * @note The internal state vector is formed as: [V, Q], where V is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
         */
        class AttitudeEKF {    
        private:

            /// @brief Timestamp of state estimation.
            int64_t stateTimestamp_;
            /// @brief Current state estimation.
            VCTR::Math::Vector<float, 7> x_;
            /// @brief Current state estimation covariance.
            VCTR::Math::Matrix<float, 7, 7> p_;
            /// @brief Prediction covariance.
            VCTR::Math::Matrix<float, 7, 7> q_;

        public:

            /**
             * @brief Standard constructor. Sets state to 0 and covariance to 1000 as starting values.
             */
            AttitudeEKF();

            /**
             * @brief Predicts system state upto given time.
             * @param time Defaults to current time.
             */
            void predict(int64_t time = VCTR::Core::NOW());

            /**
             * @brief Updates the current attitude estimation using an obvervation of the angular velocity in body frame. Usually the output of a gyroscope.
             * @param input Angular velocity in body frame.
             * @param inputCov Angular velocity covariance in body frame.
             */
            void updateBodyAngVel(const VCTR::Math::Vector<float, 3>& input, const VCTR::Math::Matrix<float, 3, 3>& inputCov);

            /**
             * @brief Updates the current attitude estimation using an obvervation of a vector in body frame and its expected position in the reference frame. 
             * @note For accerlerometers: body frame is accelerometer output and reference is gravity pointing up.
             * @note For magnetometers: body frame is magnetometer output and reference is north. These should be projected to world XY-Plane.
             * @param input Observated direction and covariance.
             */
            void updateBodyDirection(const VCTR::Math::Vector<float, 3>& input, const VCTR::Math::Matrix<float, 3, 3>& inputCov, VCTR::Math::Vector<float, 3>& reference);

            /**
             * Sets the expected process noise covariance for the model prediction.
             * @param Noise Covariance matrix.
            */
            void setProcessNoise(const VCTR::Math::Matrix<float, 7, 7>& noise);

            /**
             * @note The internal state vector is formed as: [V, Q], where V is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             * @returns estimation for state.
            */
            const VCTR::Math::Vector<float, 7>& getState();

            /**
             * @note The internal state vector is formed as: [V, Q], where V is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             * @returns estimation for covariance.
            */
            const VCTR::Math::Matrix<float, 7, 7>& getCovariance();

            /**
             * @brief Sets the current estimation for the state and covariance.
             * @note The internal state vector is formed as: [V, Q], where V is the angular velocity vector and Q is a unit quaternion rotation from the reference frame to body frame.
             * @param state The state vector and covariance to be used.
            */
            void setState(const ValueCov<VCTR::Math::Vector<float, 7>, VCTR::Math::Matrix<float, 7, 7>>& state);

        protected:

        };

        AttitudeEKF::AttitudeEKF() {
            setState(ValueCov<VCTR::Math::Vector<float, 7>, VCTR::Math::Matrix<float, 7, 7>>{0, 1000});
            q_ = 1;
            stateTimestamp_ = VCTR::Core::NOW();
        }
        
        void AttitudeEKF::predict(int64_t time) {
            
            float dTime = static_cast<float>(time - stateTimestamp_)/VCTR::Core::NANOSECONDS; 

            VCTR::Math::Quat<float> quat = x_.block<4, 1>(3, 0);

            quat = VCTR::Math::Quat<float>::fromRotVec(x_.block<3, 1>(0, 0)) * quat;

            x_.block(quat, 3, 0);

        }

        void AttitudeEKF::updateBodyAngVel(const VCTR::Math::Vector<float, 3>& input, const VCTR::Math::Matrix<float, 3, 3>& inputCov) {

            VCTR::Math::Quat<float> quat = x_.block<4, 1>(3, 0);
            
            //Transform from body to reference frame.
            auto angVel = quat.rotate(input);
            auto angVelCov = quat.rotate(inputCov);

            

        }

        void AttitudeEKF::updateBodyDirection(const VCTR::Math::Vector<float, 3>& input, const VCTR::Math::Matrix<float, 3, 3>& inputCov, VCTR::Math::Vector<float, 3>& reference) {
            
        }

        void AttitudeEKF::setProcessNoise(const VCTR::Math::Matrix<float, 7, 7>& noise) {
            q_ = noise;
        }

        const VCTR::Math::Vector<float, 7>& AttitudeEKF::getState() {
            return x_;
        }

        const VCTR::Math::Matrix<float, 7, 7>& AttitudeEKF::getCovariance() {
            return p_;
        }

        void AttitudeEKF::setState(const ValueCov<VCTR::Math::Vector<float, 7>, VCTR::Math::Matrix<float, 7, 7>>& state) {
            x_ = state.val;
            p_ = state.cov;
        }

    }
}

#endif