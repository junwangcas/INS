//
// Created by nvidia on 18-10-7.
//

#ifndef OPTIMIZATION_GPSIMU_OPTIMIZER_H
#define OPTIMIZATION_GPSIMU_OPTIMIZER_H
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include "struct.h"

using namespace Eigen;
using namespace std;

class optimizer {
public:
    optimizer();
    void run_optimize(vector< Vector8d>& variable_list, vector< Vector9d> nEdgeLst);
    int id_glocal_to_local(int id_global, int type);
};


#endif //OPTIMIZATION_GPSIMU_OPTIMIZER_H
