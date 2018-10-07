//
// Created by nvidia on 18-8-31.
//

#ifndef PROJECT_FACTOR_PTS_OBSV_H
#define PROJECT_FACTOR_PTS_OBSV_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include "MapMatcher.h"

template<typename T>
Eigen::Matrix<T,2,2> to_rotation_tmp(T yaw) {
    Eigen::Matrix<T,2,2> m1 = Eigen::Matrix<T,2,2>::Identity();
    Eigen::Matrix<T,2,2> m2 = Eigen::Matrix<T,2,2>::Identity();
    m2 << static_cast<T>(0.0), static_cast<T>(-1.0), static_cast<T>(1.0), static_cast<T>(0.0);
    Eigen::Matrix<T,2,2> R = cos(yaw) * m1 + sin(yaw) * m2;
    return R;
};
template <typename T>
T dist_pts_line_tmp(Eigen::Matrix<T,2,1> npts0, Eigen::Matrix<T,2,1> npts1,
                    Eigen::Matrix<T,2,1> npts2) {
    T x0 = npts0[0];
    T y0 = npts0[1];
    T x1 = npts1[0];
    T y1 = npts1[1];
    T x2 = npts2[0];
    T y2 = npts2[1];

    T c1 = abs(((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1));
    T c2 = sqrt(pow((y2 - y1), 2) + pow((x2 - x1), 2));
    T dist = c1 / c2;
    return dist;
}

namespace loc {
namespace map_match {
class FactorPtsObsv {
public:
    FactorPtsObsv();
    template <typename T> bool operator()(const T* const pose_data_xy,T* residual) const {
        Eigen::Matrix<T,2,1> t(pose_data_xy[0],pose_data_xy[1]);
        Eigen::Matrix<T,2,2> r = to_rotation_tmp(pose_data_xy[2]);
        Eigen::Matrix<T,2,1> _pts_0_t(static_cast<T>(_pts_0[0]),static_cast<T>(_pts_0[1]));
        Eigen::Matrix<T,2,1> _pts_1_t(static_cast<T>(_pts_1[0]),static_cast<T>(_pts_1[1]));
        Eigen::Matrix<T,2,1> _pts_2_t(static_cast<T>(_pts_2[0]),static_cast<T>(_pts_2[1]));
        Eigen::Matrix<T,2,1> p0g = r * _pts_0_t + t;
        T dist = dist_pts_line_tmp(p0g, _pts_1_t, _pts_2_t);
        residual[0] = dist;
        return true;
    }
    //void set_yaw(double yaw);
    //double get_yaw();
    void set_pts_obs(Eigen::Vector2d npts0, Eigen::Vector2d npts1, Eigen::Vector2d npts2);

private:
    //double _yaw;
    Eigen::Vector2d _pts_0, _pts_1, _pts_2;
};

}
}




#endif //PROJECT_FACTOR_PTS_OBSV_H
