//
// Created by nvidia on 18-10-7.
//
#include <ceres/ceres.h>
#include "factors.h"
#include "optimizer.h"

optimizer::optimizer() {

}

void optimizer::run_optimize(vector<Vector8d> &variable_list, vector<Vector9d> nEdgeLst) {
    // define ceres problem
    ceres::Problem optimize_problem;
    // define the variable list;
    vector< Vector6d > imubias_varibles;
    vector< Vector3d > R_variables;
    vector< Vector3d > T_variables;
    vector< Vector3d > Velocity_variables;

    for (int i = 0; i < variable_list.size(); i++){
        Vector8d variable_val = variable_list[i];
        // imu bias variable;
        if (variable_val[0] == 1){
            Vector6d imu_variable = variable_val.block<6,1>(2,0);
            imubias_varibles.push_back(imu_variable);
        }else if (variable_val[1] == 2){
            // R variables;
            Vector3d R_variable = variable_val.block<3,1>(2,0);
            R_variables.push_back(R_variable);
        }else if (variable_val[1] == 3){
            // T variables;
            Vector3d T_variable = variable_val.block<3,1>(2,0);
            T_variables.push_back(T_variable);
        }else if (variable_val[1] == 4){
            // velocity variables;
            Vector3d Velocity_variable = variable_val.block<3,1>(2,0);
            Velocity_variables.push_back(Velocity_variable);
        }
    }

    // define the edges;
    for (int i = 0; i < nEdgeLst.size(); i++){
        Vector9d nEdgeVal = nEdgeLst[i];
        int nIdEdge = int(nEdgeVal[0]);
        int nIdType = int(nEdgeVal[1]);

        if (nIdType == 1){
            // add gps factor;
            factor_GPS* factor_gps = new factor_GPS();
            Vector3d measure = nEdgeVal.block<3,1>(3,0);
            int id_T_local = id_glocal_to_local(int(nEdgeVal[2]),3);
            Vector3d& T_variable = T_variables[id_T_local];
            factor_gps->set_measure(measure);
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<factor_GPS,3,3>(factor_gps);
            optimize_problem.AddResidualBlock(cost_function,NULL,T_variable.data());
        } else if (nIdType == 2){
            // add velocity factor;
            factor_velocity* my_factor_velocity = new factor_velocity();
            Vector3d measure = nEdgeVal.block<3,1>(4,0);
            int id_R_local = id_glocal_to_local(int(nEdgeVal[2]),2);
            int id_T_local = id_glocal_to_local(int(nEdgeVal[3]),3);
            Vector3d& R_variable = R_variables[id_R_local];
            Vector3d& T_variable = T_variables[id_T_local];
            my_factor_velocity->set_measure(measure);
            //ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<factor_velocity,3,3,3>(my_factor_velocity);
            //optimize_problem.AddResidualBlock(cost_function,NULL,R_variable.data(),T_variable.data());
        } else if (nIdType == 3){
            // add acc factor;
            factor_acc* my_factor_acc = new factor_acc();
            Vector3d measure = nEdgeVal.block<3,1>(6,0);
            int id_R_local = id_glocal_to_local(int(nEdgeVal[2]),2);
            int id_T_local = id_glocal_to_local(int(nEdgeVal[3]),3);
            int id_V_local = id_glocal_to_local(int(nEdgeVal[4]),4);
            int id_T2_local = id_glocal_to_local(int(nEdgeVal[5]),3);
            Vector3d& R_variable = R_variables[id_R_local];
            Vector3d& T_variable = T_variables[id_T_local];
            Vector3d& V_variable = Velocity_variables[id_V_local];
            Vector3d& T2_variable = T_variables[id_T2_local];
            Vector6d& imubias_variable = imubias_varibles[0];
            my_factor_acc->set_measure(measure);
//            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<factor_acc,3,3,3,3,6,3>(my_factor_acc);
//            optimize_problem.AddResidualBlock(cost_function,NULL,R_variable.data(),T_variable.data(),V_variable.data(),
//                                        T2_variable.data(),imubias_variable.data());
        } else if (nIdType == 4){
            // add gyro factor;
            factor_gyro* my_factor_gyro = new factor_gyro();
            Vector3d measure = nEdgeVal.block<3,1>(4,0);
            int id_R_local = id_glocal_to_local(int(nEdgeVal[2]),2);
            int id_R2_local = id_glocal_to_local(int(nEdgeVal[3]),2);
            Vector3d& R_variable = R_variables[id_R_local];
            Vector3d& R2_variable = R_variables[id_R2_local];
            Vector6d& imubias_variable = imubias_varibles[0];
            my_factor_gyro->set_measure(measure);
            //ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<factor_gyro,3,3,3,6>(my_factor_gyro);
           // optimize_problem.AddResidualBlock(cost_function,NULL,R_variable.data(),R2_variable.data(),imubias_variable.data());
        } else if (nIdType == 5){
            // add motion model factor;
            factor_motionmodel* my_factor_motion = new factor_motionmodel();
            int id_T_local = id_glocal_to_local(int(nEdgeVal[2]),3);
            int id_V_local = id_glocal_to_local(int(nEdgeVal[3]),4);
            int id_T2_local = id_glocal_to_local(int(nEdgeVal[4]),3);
            Vector3d& T_variable = T_variables[id_T_local];
            Vector3d& V_variable = Velocity_variables[id_V_local];
            Vector3d& T2_variable = T_variables[id_T2_local];
            //ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<factor_motionmodel,3,3,3,3>(my_factor_motion);
            //optimize_problem.AddResidualBlock(cost_function,NULL,T_variable.data(),V_variable.data(),T2_variable.data());
        }
    }
    ceres::Solver::Options optimize_option;
    optimize_option.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary optimize_summary;
    ceres::Solve(optimize_option,&optimize_problem,&optimize_summary);
    std::cout<<optimize_summary.BriefReport()<<"\n";

    int id_R = 0,id_T = 0,id_V = 0;
    for (int i = 0; i < variable_list.size(); i++){
        Vector8d& variable_val = variable_list[i];
        // imu bias variable;
        if (variable_val[0] == 1){
            variable_val.block<6,1>(2,0) = imubias_varibles[0];
        }else if (variable_val[1] == 2){
            // R variables;
            variable_val.block<3,1>(2,0) = R_variables[id_R];
            id_R ++;
        }else if (variable_val[1] == 3){
            // T variables;
            variable_val.block<3,1>(2,0) = T_variables[id_T];
            id_T ++;
        }else if (variable_val[1] == 4){
            // velocity variables;
            variable_val.block<3,1>(2,0) = Velocity_variables[id_V];
            id_V ++;
        }
    }
}

int optimizer::id_glocal_to_local(int id_global, int type) {
    int id_local = -1;
    // imu bias node;
    if (type == 1){
        id_local = 1 ;
    } else {
        id_local =  ceil( (id_global - 1) / 3);
    }
    return id_local - 1;
}