//
// Created by nvidia on 18-9-30.
//

#ifndef OPTIMIZATION_GPSIMU_MATH_UTILITY_H
#define OPTIMIZATION_GPSIMU_MATH_UTILITY_H

#include <Eigen/Core>

class math_utility {
public:
    template <typename T>
    static Eigen::Matrix<T,3,3> to_rotation(T a1,T a2, T a3);


    template<typename T>
    Eigen::Matrix<T,2,2> to_rotation_tmp(T yaw) {
        Eigen::Matrix<T,2,2> m1 = Eigen::Matrix<T,2,2>::Identity();
        Eigen::Matrix<T,2,2> m2 = Eigen::Matrix<T,2,2>::Identity();
        m2 << static_cast<T>(0.0), static_cast<T>(-1.0), static_cast<T>(1.0), static_cast<T>(0.0);
        Eigen::Matrix<T,2,2> R = cos(yaw) * m1 + sin(yaw) * m2;
        return R;
    };

};


#endif //OPTIMIZATION_GPSIMU_MATH_UTILITY_H
