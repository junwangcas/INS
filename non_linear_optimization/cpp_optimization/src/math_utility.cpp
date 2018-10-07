//
// Created by nvidia on 18-9-30.
//

#include "math_utility.h"

template <typename T>
Eigen::Matrix<T,3,3> math_utility::to_rotation(T yaw, T pitch, T roll) {
    Eigen::Matrix<T,3,3> Rx,Ry,Rz,R;
    Rz << cos(yaw), -sin(yaw),0, sin(yaw),cos(yaw),0, 0, 0, 1;
    Ry << cos(pitch), 0, sin(pitch),0,1,0,-sin(pitch),0,cos(pitch);
    Rx << 1,0,0,0,cos(roll),-sin(roll),0,sin(roll),cos(roll);
    R = Rz*Ry*Rx;
    return R;
}