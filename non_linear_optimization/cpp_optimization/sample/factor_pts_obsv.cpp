//
// Created by nvidia on 18-8-31.
//

#include "mapmatch/factor_pts_obsv.h"

namespace loc {
namespace map_match {


FactorPtsObsv::FactorPtsObsv() {

}
//double FactorPtsObsv::get_yaw() {
//    return _yaw;
//}
//void FactorPtsObsv::set_yaw(double yaw) {
//    _yaw = yaw;
//}
void FactorPtsObsv::set_pts_obs(Eigen::Vector2d npts0, Eigen::Vector2d npts1, Eigen::Vector2d npts2) {
    _pts_0 = npts0;
    _pts_1 = npts1;
    _pts_2 = npts2;
}

}
}
