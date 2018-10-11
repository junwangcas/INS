//
// Created by nvidia on 18-9-30.
//

#ifndef OPTIMIZATION_GPSIMU_FACTORS_H
#define OPTIMIZATION_GPSIMU_FACTORS_H

#include <Eigen/Core>
#include "math_utility.h"

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

public:
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
        Eigen::Matrix<T,3,1> acc_measure_ei(static_cast<T>(_acc_measure[0]), static_cast<T>(_acc_measure[1]), static_cast<T>(_acc_measure[2]));
        Eigen::Matrix<T,3,1> gravity_ei(static_cast<T>(_gravity[0]),static_cast<T>(_gravity[1]),static_cast<T>(_gravity[2]));
        Eigen::Matrix<T,3,1> acc_true = acc_measure_ei - bias_ei - R1.transpose()*gravity_ei;
        Eigen::Matrix<T,3,1> T1(pose_t1[0],pose_t1[1],pose_t1[2]);
        Eigen::Matrix<T,3,1> T2(pose_t2[0],pose_t2[1],pose_t2[2]);
        Eigen::Matrix<T,3,1> V1(velocity[0],velocity[1],velocity[2]);

        Eigen::Matrix<T,3,1> T2_assum = T1 + V1*static_cast<T>(_unit_time)+ T(0.5)*acc_true*static_cast<T>(_unit_time)*static_cast<T>(_unit_time);
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
public:
    factor_gyro();
    template<typename T>
    bool operator()(const T* const pose_angle1,const T* const pose_angle2,const T* const bias,
            T* residual){
        Eigen::Matrix<T,3,1> pose_angle1_ei(pose_angle1[0],pose_angle1[1],pose_angle1[2]);
        Eigen::Matrix<T,3,1> pose_angle2_ei(pose_angle2[0],pose_angle2[1],pose_angle2[2]);
        Eigen::Matrix<T,3,1> bias_ei(bias[3],bias[4],bias[5]);

        Eigen::Matrix<T,3,1> pose_angle2_assum = pose_angle1_ei + (T(_gyro_measure)-bias_ei)*_unit_time;
        residual[0] =  math_utility::wrap_PI(pose_angle2_ei[0] - pose_angle2_assum[0]);
        residual[1] =  math_utility::wrap_PI(pose_angle2_ei[1] - pose_angle2_assum[1]);
        residual[2] =  math_utility::wrap_PI(pose_angle2_ei[2] - pose_angle2_assum[2]);
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
//////////////////motion factor/////////////////////
class factor_motionmodel{
public:
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
