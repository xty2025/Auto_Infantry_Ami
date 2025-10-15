#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "interfaceType.hpp"

namespace predictor{
    const int N_X = 10;
    const int N_Y = 4;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    //state: x,y,theta,vx,vy,omega,r1,r2,z1,z2
    struct stateTransFunc{
        template<typename T>
        void operator()(const T& x, T& xp){
            xp[0] = x[0] + x[3] * dt;
            xp[1] = x[1] + x[4] * dt;
            xp[2] = x[2] + x[5] * dt;
            xp[3] = x[3];
            xp[4] = x[4];
            xp[5] = x[5];
            xp[6] = x[6];
            xp[7] = x[7];
            xp[8] = x[8];
            xp[9] = x[9];
        }
        double dt;
        void setDt(double dt){
            this->dt = dt * 50.0;//估计为20ms，乘以系数保证dt≈1
        }
    };
    //measure: pitch, yaw, distance, visible
    struct measureFunc{
        template<typename T>
        void operator()(const T s[N_X], T m[N_Y]){
            T x = s[0], y = s[1], z = (id % 2 == 0) ? s[8] : s[9];
            T r = (id % 2 == 0) ? s[6] : s[7];
            T angle = s[2] + M_PI / 2 * id;
            T armor_x = x + r * ceres::cos(angle);
            T armor_y = y + r * ceres::sin(angle);
            m[2] = ceres::sqrt(armor_x*armor_x + armor_y*armor_y + z*z);
            m[0] = ceres::asin(z/m[2]);
            m[1] = ceres::atan2(armor_y, armor_x);
            T dot_product = x * ceres::cos(angle) + y * ceres::sin(angle);
            T dist = ceres::sqrt(x*x + y*y);
            dot_product = dot_product / dist;
            if(dot_product < 0) m[3] = T(1);
            else m[3] = ceres::exp(-dot_product);
        }
        int id;
        void setId(int id){
            this->id = id;
        }
    };

    class MotionModel 
    {
    public:
        void initMotionModel();
        Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid);
        void Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id);
    private:
        VectorX first_state_estimate(const VectorY& measure, const int armor_id);
        ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, N_X, N_Y> ekf;
        stateTransFunc stateTrans;
        measureFunc measure;
        bool firstUpdate = true;
    };
}