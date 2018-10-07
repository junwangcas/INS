//
// Created by nvidia on 18-8-31.
//

#include <ceres/ceres.h>
#include "mapmatch/optimizer_ceres.h"
#include "mapmatch/factor_odom.h"
#include "mapmatch/factor_pts_obsv.h"
using namespace std;
using namespace Eigen;

namespace loc {
namespace map_match {
OptimizerCeres::OptimizerCeres() {}
void  OptimizerCeres::run_optimize(vector<Vector4d>& nPoseLst, vector<Vector9d>& nEdgeObserveLst,
                                   vector<Vector6d>& nEdgeOdomLst, vector<int>& nEdgeIndicator) {

    // define ceres problem
    ceres::Problem optimize_problem;
    // define the parameters;
    vector<Vector3d> pose_params;
    for (int i = 0; i < nPoseLst.size(); i++) {
        Vector4d nposeval = nPoseLst[i];
        Vector3d pose_xytheta = Vector3d(nposeval[1],nposeval[2],nposeval[3]);
        pose_params.push_back(pose_xytheta);
    }

    // add odom cost function factors;
    for (int i = 0; i < nEdgeOdomLst.size(); i++) {
        Vector6d nEdgeOdomVal = nEdgeOdomLst[i];
        int nIdEdge = int(nEdgeOdomVal[0]);
        int nIdPose1 = int(nEdgeOdomVal[1]);
        int nIdPose2 = int(nEdgeOdomVal[2]);
        Vector3d& pose_param1 = pose_params[nIdPose1];
        Vector3d& pose_param2 = pose_params[nIdPose2];
        Vector2d nMeasure(nEdgeOdomVal(3), nEdgeOdomVal(4));
        FactorOdom* factor_odom = new FactorOdom();
        factor_odom->set_measure(nMeasure);
       // if (is_first_init){
        //    factor_odom->set_init_weight(0.001);
        //}
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FactorOdom, 3, 3, 3>(factor_odom);
        optimize_problem.AddResidualBlock(cost_function,NULL,pose_param1.data(),pose_param2.data());
    }

    // create points observe factors ;
    for (int i = 0; i < nEdgeObserveLst.size(); i++) {
        Vector9d nEdgePtsVal = nEdgeObserveLst[i];
        int nIdEdge = int(nEdgePtsVal[0]);
        int nIdPose = int(nEdgePtsVal[1]);
        Vector4d pose_val = nPoseLst[nIdPose];
        Vector2d npts0(nEdgePtsVal[2], nEdgePtsVal[3]);
        Vector2d npts1(nEdgePtsVal[4], nEdgePtsVal[5]);
        Vector2d npts2(nEdgePtsVal[6], nEdgePtsVal[7]);
        Vector3d& pose_param = pose_params[nIdPose];

        FactorPtsObsv* factor_pts_obs = new FactorPtsObsv();
        factor_pts_obs->set_pts_obs(npts0,npts1,npts2);
        //factor_pts_obs->set_yaw(pose_val[3]);
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FactorPtsObsv,1,3>(factor_pts_obs);
        optimize_problem.AddResidualBlock(cost_function,NULL,pose_param.data());
    }

    // fix the first pose;
    Vector3d& pose_val_fix = pose_params[0];
    optimize_problem.SetParameterBlockConstant(pose_val_fix.data());

    // run optimize;
    ceres::Solver::Options optimize_option;
    optimize_option.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary optimize_summary;
    ceres::Solve(optimize_option,&optimize_problem,&optimize_summary);

    _file_handle_debug.open("/home/nvidia/data/txt_data/optimize_report.txt", std::ofstream::out|std::ofstream::app);
    int nsizeprec = 5;
    if (_file_handle_debug) {
        _file_handle_debug << optimize_summary.BriefReport() << endl;
    }
    // replace the optimized values;
    for (int i = 0; i < nPoseLst.size(); i++) {
        Vector4d& nposeval = nPoseLst[i];
        Vector3d pose_xy = pose_params[i];
        nposeval[1] = pose_xy[0];
        nposeval[2] = pose_xy[1];
        nposeval[3] = pose_xy[2];
    }
}
}
}
