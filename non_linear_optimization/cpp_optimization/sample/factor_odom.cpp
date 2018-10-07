//
// Created by nvidia on 18-8-31.
//

#include "mapmatch/factor_odom.h"
using namespace Eigen;
namespace loc {
namespace map_match {
FactorOdom::FactorOdom() {

}
void FactorOdom::set_measure(Vector2d measure) {
    _measurement = measure;
}
void FactorOdom::set_init_weight(double weight) {
    _odom_init_weight = weight;
}
}
}