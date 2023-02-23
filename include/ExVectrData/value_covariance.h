#ifndef EXVECTRDATA_VALUECOVARIANCE_H
#define EXVECTRDATA_VALUECOVARIANCE_H


namespace VCTR
{

    namespace Data
    {

        /**
         * Simple container class for storing values and their covariances.
         * @note Easy constructor by -> ValueCov{value, covariance}
         */
        template <typename VALUETYPE, typename COVARIANCETYPE>
        struct ValueCov
        {
        public:
            /// @brief Values of sensor data.
            VALUETYPE val = 0;

            /// @brief Covariance of sensor values.
            COVARIANCETYPE cov = 0;

        };

    }
}

#endif