//
// Created by nvidia on 18-9-30.
//

#ifndef OPTIMIZATION_GPSIMU_FACTORS_H
#define OPTIMIZATION_GPSIMU_FACTORS_H

#include <Eigen/Core>
#include "math_utility.h"

class factors {

};

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

/////////////////////////////////////////////////
//////////////////GPS Factor/////////////////////
class factor_GPS{
public:
    factor_GPS();
    template <typename T> bool operator()(const T* const translation, T* residual)const {
        residual[0] = T(_measurement[0]) - translation[0];
        residual[1] = T(_measurement[1]) - translation[1];
        residual[2] = T(_measurement[2]) - translation[2];
        return true;
    }

private:
    Eigen::Vector3d _measurement;
    double _gps_init_weight = 1;
public:
    void set_measure(Eigen::Vector3d measure);
    void set_init_weight(double weight);
};
/////////////////////////////////////////////////
//////////////////car velocity/////////////////////
class factor_velocity{
public:
    factor_velocity();
    template <typename T> bool operator()(const T* const pose_angle, const T* const velocity, T* residual) const{
        // translate the velocity to local;
        Eigen::Matrix<T,3,3> R =  math_utility::to_rotation(pose_angle[0],pose_angle[1],pose_angle[2]);
        Eigen::Matrix<T,3,1> velocity_ei(velocity[0],velocity[1],velocity[2]);
        Eigen::Matrix<T,3,1> velocity_local = R.transpose() * velocity_ei;
        //
        residual[0] = velocity_local[0] - T(_velocity_measure[0]);
        residual[1] = velocity_local[1] - T(_velocity_measure[1]);
        residual[2] = velocity_local[2] - T(_velocity_measure[2]);
        return true;
    }

private:
    void set_measure(Eigen::Vector3d measure);
    void set_init_weight(double weight);

private:
    Eigen::Vector3d _velocity_measure;
    double _velocity_weight = 1;
};

/////////////////////////////////////////////////
//////////////////acc velocity/////////////////////
class factor_acc{
public:
    factor_acc();
    template <typename T>
    bool operator()(const T* const pose_angle1, const T* const pose_t1,
            const T* const velocity, const T* const pose_t2, const T* const bias, T* residual) const{
        // trans _gravity to local;
        Eigen::Matrix<T,3,3> R1 =  math_utility::to_rotation(pose_angle1[0],pose_angle1[1],pose_angle1[2]);
        Eigen::Matrix<T,3,1> bias_ei(bias[0],bias[1],bias[2]);
        Eigen::Matrix<T,3,1> acc_true = T(_acc_measure) - bias_ei - R1.transpose()*T(_gravity);
        Eigen::Matrix<T,3,1> T1(pose_t1[0],pose_t1[1],pose_t1[2]);
        Eigen::Matrix<T,3,1> T2(pose_t2[0],pose_t2[1],pose_t2[2]);
        Eigen::Matrix<T,3,1> V1(velocity[0],velocity[1],velocity[2]);

        Eigen::Matrix<T,3,1> T2_assum = T1 + V1*T(_unit_time)+ 0.5*acc_true*T(_unit_time)*T(_unit_time);
        residual[0] = T2_assum[0] - T2[0];
        residual[1] = T2_assum[1] - T2[1];
        residual[2] = T2_assum[2] - T2[2];
        return true;
    }

private:
    Eigen::Vector3d _gravity;
    Eigen::Vector3d _acc_measure;
    double _acc_weight = 1;
    double _unit_time = 0.01;
public:
    void set_measure(Eigen::Vector3d measure);
    void set_init_weight(double weight);
};
/////////////////////////////////////////////////
//////////////////gyro factor/////////////////////
class factor_gyro{
private:
    factor_gyro();
    template<typename T>
    bool operator()(const T* const pose_angle1,const T* const pose_angle2,const T* const bias,
            T* residual){
        Eigen::Matrix<T,3,1> pose_angle1_ei(pose_angle1[0],pose_angle1[1],pose_angle1[2]);
        Eigen::Matrix<T,3,1> pose_angle2_ei(pose_angle2[0],pose_angle2[1],pose_angle2[2]);
        Eigen::Matrix<T,3,1> bias_ei(bias[3],bias[4],bias[5]);

        Eigen::Matrix<T,3,1> pose_angle2_assum = pose_angle1_ei + (T(_gyro_measure)-bias_ei)*_unit_time;
        residual[0] = pose_angle2_ei[0] - pose_angle2_assum[0];
        residual[1] = pose_angle2_ei[1] - pose_angle2_assum[1];
        residual[2] = pose_angle2_ei[2] - pose_angle2_assum[2];
        return true;
    }
private:
    Eigen::Vector3d _gyro_measure;
    double _gyro_weight = 1;
    double _unit_time = 0.01;
public:
    void set_measure(Eigen::Vector3d measure);
    void set_init_weight(double weight);
};
/////////////////////////////////////////////////
//////////////////gyro factor/////////////////////
class factor_motionmodel{
private:
    factor_motionmodel();
    template <typename T>
    bool operator()(const T* const pose_t1, const T* const velocity, const T* const pose_t2, T* residual){
        Eigen::Matrix<T,3,1> pose_t1_ei(pose_t1[0],pose_t1[1],pose_t1[2]);
        Eigen::Matrix<T,3,1> pose_t2_ei(pose_t2[0],pose_t2[1],pose_t2[2]);
        Eigen::Matrix<T,3,1> velocity_ei(velocity[0],velocity[1],velocity[2]);

        Eigen::Matrix<T,3,1> pose_t2_assum = pose_t1_ei + velocity_ei*T(_unit_time);
        residual[0] = pose_t2_assum[0] - pose_t2_ei[0];
        residual[1] = pose_t2_assum[1] - pose_t2_ei[1];
        residual[2] = pose_t2_assum[2] - pose_t2_ei[2];
        return residual;
    }
private:
    double _motion_weight = 1;
    double _unit_time = 0.01;
public:
    void set_init_weight(double weight);
};
#endif //OPTIMIZATION_GPSIMU_FACTORS_H
