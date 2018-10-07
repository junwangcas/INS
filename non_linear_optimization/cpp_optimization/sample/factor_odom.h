//
// Created by nvidia on 18-8-31.
//

#ifndef PROJECT_FACTOR_ODOM_H
#define PROJECT_FACTOR_ODOM_H
#include <Eigen/Core>
#include "MapMatcher.h"
#include "factor_pts_obsv.h"

namespace loc {
namespace map_match {
//    template <typename T>
//    T wrap_2PI(const T radian)
//    {
//            T res = radian % (M_PI*2.0);
//            if (res < 0) {
//                    res += T(M_PI*2.0);
//            }
//            return res;
//    }
    template <typename T>
    T wrap_PI(T radian)
    {
            while (radian < T(-M_PI)){
                    radian += T(M_PI*2.0);
            }
            while (radian > T(M_PI)){
                    radian -= T(M_PI*2.0);
            }
            return radian;
    }

class FactorOdom {
public:
    FactorOdom();
    template <typename T> bool operator()(const T* const pose1_data_xy,const T* const pose2_data_xy,T* residual) const {
        Eigen::Matrix<T,2,1> t1(pose1_data_xy[0],pose1_data_xy[1]);
        Eigen::Matrix<T,2,2> R1 = to_rotation_tmp(pose1_data_xy[2]);
        Eigen::Matrix<T,2,1> t2(pose2_data_xy[0],pose2_data_xy[1]);
        Eigen::Matrix<T,2,1> delta = R1.transpose()*(t2-t1);
//Matrix<T,2,1> dif
        T weight_x = T(10.0);
        T weight_y = T(15.0);
        T weight_yaw = T(1.0e10);
        T threshold_yaw = T(5.0/57.3*5.0/57.3);
        residual[0] = T(_odom_init_weight)*weight_x*(static_cast<T>(_measurement[0]) - delta(0,0));
        residual[1] = T(_odom_init_weight)*weight_y*(static_cast<T>(_measurement[1]) - delta(1,0));
        T delta_yaw = wrap_PI<T>((pose2_data_xy[2] - pose1_data_xy[2]))*wrap_PI<T>((pose2_data_xy[2] - pose1_data_xy[2]));

        if (delta_yaw > threshold_yaw){
             //   std::cout<<"delta_yaw:"<< delta_yaw<<" deltaxy:"<<delta(0,0)<<std::endl;
                residual[2] = delta_yaw*weight_yaw;
                //exit(-1);
        }else{
                residual[2] = T(0.0);
        }
        return true;
    }
    void set_measure(Eigen::Vector2d measure);
    void set_init_weight(double weight);

private:
    Eigen::Vector2d _measurement;
    double _odom_init_weight = 1;
};


}
}


#endif //PROJECT_FACTOR_ODOM_H
