#ifndef EXVECTRDATA_VALUECOVARIANCE_H
#define EXVECTRDATA_VALUECOVARIANCE_H

#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"

namespace VCTR
{

    namespace Data
    {

        /**
         * Simple container class for storing values and their covariances.
         * @note Easy constructor by -> ValueCov{value, covariance}
         * @tparam TYPE Value type. Float, double, int etc.
         * @tparam NUMVALUES Number of values. Basically length of value vector and width height of covariance matrix.
         */
        template <typename TYPE, size_t NUMVALUES>
        struct ValueCov
        {
        public:
            /// @brief Values of sensor data.
            Math::Vector<TYPE, NUMVALUES> val = 0;

            /// @brief Covariance of sensor values.
            Math::Matrix<TYPE, NUMVALUES, NUMVALUES> cov = 0;
        };

    }
}

#endif