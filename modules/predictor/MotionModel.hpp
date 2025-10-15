#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "type.hpp"
#include "Log/log.hpp"
namespace predictor{


const int N_X = 12;
//预测状态12维度
const int N_Y = 7;
//观测7
using VectorX = Eigen::Matrix<double, N_X, 1>;
using VectorY = Eigen::Matrix<double, N_Y, 1>;
using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;


template<typename T>
inline T remainder_PI(T a)
{
    return a - 2.0 * M_PI * ceres::floor(a / M_PI / 2.0 + 0.5);
}

template<typename T>
inline T min(T a, T b)
{
    //return a < b ? a : b;
    return remainder_PI(a - b) < 0.0 ? a : b;
}
template<typename T>
inline T max(T a, T b)
{
    //return a > b ? a : b;
    return remainder_PI(a - b) > 0.0? a : b;
}

//state: x,vx,y,vy,theta,omega,r1,r2,z1,z2, ax,ay
struct stateTransFunc{
    template<typename T>
    void operator()(const T& x, T& xp){
        xp[0] = x[0] + x[1] * dt;
        xp[1] = x[1] + x[10] * dt; // ax
        xp[2] = x[2] + x[3] * dt;
        xp[3] = x[3] + x[11] * dt; // ay
        xp[4] = x[4] + x[5] * dt;
        xp[5] = x[5];
        
        xp[6] = x[6];
        xp[7] = x[7];
        xp[8] = x[8];
        xp[9] = x[9];
        xp[10] = x[10];
        xp[11] = x[11];
    }
    double dt;
    void setDt(double dt){
        this->dt = dt;
    }
};//状态矩阵
//measure: ax,ay,az,tangent,angle_left,angle_right
//new measure: armor_pitch, armor_yaw, dist, tangent, armor_left, armor_right
//观测矩阵
struct measureFunc{
    template<typename T>
    void operator()(const T s[N_X], T m[N_Y]){

        T theta = s[4] + M_PI / 2 * id;
        T x = s[0];
        T y = s[2];
        T z = (id % 2 == 0) ? s[8] : s[9];//controller中的装甲板奇偶的转化逻辑
        T r = (id % 2 == 0) ? s[6] : s[7];
        T armor_x = x + r * ceres::cos(theta);
        T armor_y = y - r * ceres::sin(theta);
        T r1 = s[6];
        T r2 = s[7];
//        m[0] = armor_x;
//        m[1] = armor_y;
//        m[2] = z;
        m[2] = ceres::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
        m[0] = ceres::atan2(z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
        m[1] = ceres::atan2(armor_y, armor_x);
        m[3] = M_PI/2 - theta;
        m[6] = ceres::atan2(y,x);
        T true_theta = s[4];
        T angle[4];
        angle[0] =  ceres::atan2(y - r1 * ceres::sin(true_theta) + r2 * ceres::cos(true_theta),x + r1 * ceres::cos(true_theta) + r2 * ceres::sin(true_theta)) ;/// M_PI;
        angle[1] =  ceres::atan2(y - r1 * ceres::sin(true_theta) - r2 * ceres::cos(true_theta),x + r1 * ceres::cos(true_theta) - r2 * ceres::sin(true_theta)) ;/// M_PI;
        angle[2] =  ceres::atan2(y + r1 * ceres::sin(true_theta) - r2 * ceres::cos(true_theta),x - r1 * ceres::cos(true_theta) - r2 * ceres::sin(true_theta)) ;/// M_PI;
        angle[3] =  ceres::atan2(y + r1 * ceres::sin(true_theta) + r2 * ceres::cos(true_theta),x - r1 * ceres::cos(true_theta) + r2 * ceres::sin(true_theta)) ;/// M_PI;
        
        if(this->total_id1 == -1 || mode == false)
        {
            m[4] = min(min(angle[0], angle[1]), min(angle[2], angle[3]));
            m[5] = max(max(angle[0], angle[1]), max(angle[2], angle[3]));
            // if(mode == true)
            // std::cout << "enter no id mode"<<std::endl;
        }
        else if(this->total_id2 == -1)
        {
            m[4] = min(angle[this->total_id1], angle[(this->total_id1 + 3)%4]);
            m[5] = max(angle[(this->total_id1 + 1)%4], angle[(this->total_id1 + 2)%4]);
            // std::cout << "enter one id mode"<<std::endl;
            // std::cout << "calc id:" <<
            //     (m[3] == angle[this->total_id1] ? this->total_id1 : (this->total_id1 + 3)%4) <<
            //     " " <<
            //     (m[4] == angle[(this->total_id1 + 1)%4] ? (this->total_id1 + 1)%4 : (this->total_id1 + 2)%4) << std::endl;
        }
        else
        {
            m[4] = angle[this->total_id1];
            m[5] = angle[(this->total_id2 + 1)%4];
            //std::cout << "enter two id mode"<<std::endl;
            //std::cout<< "calc id:" << this->total_id1 << " " << (this->total_id2 + 1)%4 << std::endl;
        }
    }
    int id;
    int total_id1 = -1;
    int total_id2 = -1;
    bool mode = false;
    void setId(int id_){
        this->id = id_;
    }
    //mode = true: Update Mode
    //mode = false: Predict Mode 
    void setMode(bool updateMode){
        this->mode = updateMode;
    }
    //ATENTION: must satisfise (id2 - id1) % 4 == 1 if id2 != -1
    void setVisibleId(int id1, int id2=-1){
        if(id2 != -1)
        {
            if((id1 - id2 + 4) % 4 == 1)
                std::swap(id1, id2);
            if((id2 - id1 + 4) % 4 != 1)
            {
                std::cout << "id2 - id1 != 1" << std::endl;
                ERROR("id2 - id1 != 1 in setVisibleId");
                //throw std::runtime_error("id2 - id1 != 1 in setVisibleId");
            }
        }
        this->total_id1 = id1;
        this->total_id2 = id2;
    }
    void resetVisibleId(){
        this->total_id1 = -1;
        this->total_id2 = -1;
    }
};

class MotionModel 
{
public:
    void initMotionModel();
    VectorX getPredictResult(const Time::TimeStamp& timestamp);
    VectorY measureFromState(const VectorX& state, int armor_id)//0，1，2，3
    {
        VectorY measure_vec;
        measure.setMode(false);
        measure.setId(armor_id);
        measure(state.data(), measure_vec.data());
        return measure_vec;
    }
    void Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id);
    bool Stable() const { return whole_car_stable; }
    bool armorStable() const { return armor_stable; }
    void setUpdateTotalId(int id1, int id2=-1){
        this->id1 = id1;
        this->id2 = id2;
    }
    void reset(void){
        firstUpdate = true;
        id1 = -1;
        id2 = -1;
    }
private:
    bool whole_car_stable = true;
    bool armor_stable = false;
    int id1 = -1;
    int id2 = -1;
    VectorX first_state_estimate(const VectorY& measure, const int armor_id)
    {
        VectorX state;
        state.setZero();

        // 观测点坐标（圆上点）、切线角度
        //double x_obs   = measure[0];
        //double y_obs   = measure[1];
        //double z_obs   = measure[2];
        //转换成世界坐标。
        double dist = measure[2];
        double z_obs = dist * std::sin(measure[0]);  // armor_pitch
        double x_obs = dist * std::cos(measure[0]) * std::cos(measure[1]);  // armor_yaw
        double y_obs = dist * std::cos(measure[0]) * std::sin(measure[1]);  // armor_yaw

        double tangent = measure[3];
        // 反算朝向 theta: m[2] = π/2 - (theta + π/2*id) => theta = π/2 - π/2*id - tangent
        double theta = M_PI/2 - M_PI/2*armor_id - tangent;
        // 假设常量半径
        double r = 0.3;
        // 计算圆心
        double ang = theta + M_PI/2*armor_id;
        state[0] = x_obs - r * std::cos(ang);  // cx
        state[2] = y_obs + r * std::sin(ang);  // cy
        // 设置朝向和半径
        state[4] = theta;
        state[6] = r;  // r1
        state[7] = r;  // r2
        state[8] = z_obs;  // z1
        state[9] = z_obs;  // z2
        state[10] = state[11] = 0;
        return state;
    }
    ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, N_X, N_Y> ekf;
    stateTransFunc stateTrans;
    measureFunc measure;
    bool firstUpdate = true;
};

}