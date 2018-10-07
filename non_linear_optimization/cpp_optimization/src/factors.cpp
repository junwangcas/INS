//
// Created by nvidia on 18-9-30.
//

#include "factors.h"

/////////////////////////////////////////////////
//////////////////GPS Factor/////////////////////
void factor_GPS::set_init_weight(double weight) {
    _gps_init_weight = weight;
}

void factor_GPS::set_measure(Eigen::Vector3d measure) {
    _measurement = measure;
}


/////////////////////////////////////////////////
//////////////////acc velocity/////////////////////
void factor_acc::set_init_weight(double weight) {
    _acc_weight = weight;
}
void factor_acc::set_measure(Eigen::Vector3d measure) {
    _acc_measure = measure;
    _gravity << 0, 0, -9.80665;
}

