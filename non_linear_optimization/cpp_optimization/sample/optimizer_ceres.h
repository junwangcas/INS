//
// Created by nvidia on 18-8-31.
//

#ifndef PROJECT_OPTIMIZER_CERES_H
#define PROJECT_OPTIMIZER_CERES_H

#include <Eigen/Dense>
#include <vector>
#include <fstream>

using namespace Eigen;
using namespace std;

namespace loc {
namespace map_match {
typedef Matrix<double, 1, 1> Vector1d;
typedef Matrix<double, 4, 1> Vector4d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 1, 1> Matrix1d;

class OptimizerCeres {
public:
    OptimizerCeres();
    void run_optimize(vector<Vector4d>& nPoseLst, vector<Vector9d>& nEdgeObserveLst,
                      vector<Vector6d >& nEdgeOdomLst, vector<int>& nEdgeIndicator);

private:
    double _weight_odom = 0.001;
    std::ofstream _file_handle_debug;
};
}
}

#endif //PROJECT_OPTIMIZER_CERES_H
