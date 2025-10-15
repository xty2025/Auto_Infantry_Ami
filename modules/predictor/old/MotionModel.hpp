#pragma once
#include "optimizeSolve.hpp"
#include "interfaceType.hpp"

using namespace StateFitting;
class MotionModel : public StateFitting::OptimizeSolve
{
public:
    const double PI = 3.1415926;
    void initMotionModel(void)
    {
        Eigen::VectorXd min_bound(10);
        Eigen::VectorXd max_bound(10);
        Eigen::VectorXd init_bound(10);
        min_bound << -20, -20, -1e9, -5, -5, -5*PI, 0.2, 0.2, -1, -1;
        max_bound << 20, 20, 1e9, 5, 5, 5*PI, 0.4, 0.4, 1, 1;
        init_bound << 0, 0, 0, 0, 0, 0, 0.3, 0.3, 0, 0;//很可能会被first_state_estimate覆盖
        Eigen::Matrix3Xd bound(3, 10);
        bound.row(0) = min_bound.transpose();
        bound.row(1) = max_bound.transpose();
        bound.row(2) = init_bound.transpose();
        this->setBoundInit(bound);
        this->setHistorySizeAndLoss(0.1, 0.01, 100, 5);

        Eigen::VectorXd state_coef(10);
        Eigen::VectorXd measure_coef(4);
        state_coef << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
        measure_coef << 10, 10, 1, 1;
        this->setStateCoef(state_coef);
        this->setMeasureCoef(measure_coef);
    }
    //x,y,omega,vx,vy,alpha,r1,r2,h1,h2
    ADvector ideal_stateUpdate(const ADvector &state, const double delta_time) override
    {
        ADvector state_update(10);
        state_update[0] = state[0] + state[3] * delta_time;
        state_update[1] = state[1] + state[4] * delta_time;
        state_update[2] = state[2] + state[5] * delta_time;
        state_update[3] = state[3];
        state_update[4] = state[4];
        state_update[5] = state[5];
        state_update[6] = state[6];
        state_update[7] = state[7];
        state_update[8] = state[8];
        state_update[9] = state[9];
        return state_update;
    }
    Eigen::VectorXd ideal_stateUpdate(const Eigen::VectorXd &state, const double delta_time) override
    {
        Eigen::VectorXd state_update(10);
        state_update[0] = state[0] + state[3] * delta_time;
        state_update[1] = state[1] + state[4] * delta_time;
        state_update[2] = state[2] + state[5] * delta_time;
        state_update[3] = state[3];
        state_update[4] = state[4];
        state_update[5] = state[5];
        state_update[6] = state[6];
        state_update[7] = state[7];
        state_update[8] = state[8];
        state_update[9] = state[9];
        return state_update;
    }
    //p,y,distance,visible
    //measure_id allow 0,1,2,3
    //0,2 ~ r1,h1 1,3 ~ r2,h2
    typedef CppAD::AD<double> ADdouble;
    ADvector ideal_measure(const ADvector &state, const int measure_id) override
    {
        ADvector measure(4);
        //1. calc xyz
        ADdouble x = state[0], y = state[1], z = (measure_id % 2 == 0) ? state[8] : state[9];
        ADdouble r = (measure_id % 2 == 0) ? state[6] : state[7];
        ADdouble angle = state[2] + PI / 2 * measure_id;
        ADdouble armor_x = x + r * CppAD::cos(angle);
        ADdouble armor_y = y + r * CppAD::sin(angle);
        //2. change xyz to pyd and calc visible
        measure[2] = CppAD::sqrt(armor_x*armor_x + armor_y*armor_y + z*z);
        measure[0] = CppAD::asin(z/measure[2]);  // pitch
        measure[1] = CppAD::atan2(armor_y, armor_x);   // yaw
        ADdouble dot_product = x * CppAD::cos(angle) + y * CppAD::sin(angle);
        measure[3] = CppAD::CondExpLt(dot_product, ADdouble(0.0), ADdouble(1.0), ADdouble(0.0));
        return measure;
    }
    Eigen::VectorXd ideal_measure(const Eigen::VectorXd &state, const int measure_id) override
    {
        Eigen::VectorXd measure(4);
        //1. calc xyz
        double x = state[0], y = state[1], z = (measure_id % 2 == 0) ? state[8] : state[9];
        double r = (measure_id % 2 == 0) ? state[6] : state[7];
        double angle = state[2] + PI / 2 * measure_id;
        double armor_x = x + r * cos(angle);
        double armor_y = y + r * sin(angle);
        //2. change xyz to pyd and calc visible
        measure[2] = sqrt(armor_x*armor_x + armor_y*armor_y + z*z);
        measure[0] = asin(z/measure[2]);  // pitch
        measure[1] = atan2(armor_y, armor_x);   // yaw
        double dot_product = x * cos(angle) + y * sin(angle);
        measure[3] = dot_product < 0 ? 1 : 0;
        return measure;
    }

    Dvector first_state_estimate(const Dvector& measure, const int measure_id) override
    {
        if(measure[3] == 0) WARN("First state estimate with invisible armor, may cause error");
        Dvector state(10);
        state[0] = measure[2] * cos(measure[0]) * cos(measure[1]);
        state[1] = measure[2] * cos(measure[0]) * sin(measure[1]);
        state[2] = PI;
        state[3] = 0;
        state[4] = 0;
        state[5] = 0;
        state[6] = 0.3;
        state[7] = 0.3;
        state[8] = -0.05;
        state[9] = -0.05;
        state[8 + measure_id % 2] = measure[2] * sin(measure[0]);
        return state;
    }

    Prediction getPredictResult(Time::TimeStamp timestamp, int carid)
    {
        lockState();
        Prediction result;
        double dt = timestamp2dt(timestamp);
        auto nowstate = getCurrentState();
        nowstate = ideal_stateUpdate(nowstate, dt);
        result.id = carid;
        result.x = nowstate[0];
        result.y = nowstate[1];
        result.theta = nowstate[2];
        result.r1 = nowstate[6];
        result.r2 = nowstate[7];
        result.z0 = nowstate[8];
        result.z1 = nowstate[9];
        for (int i = 0; i < 4; i++)
        {
            auto armormeasure = ideal_measure(nowstate, i);
            double p = armormeasure[0], y = armormeasure[1], d = armormeasure[2];
            result.armors[i].id = i;
            result.armors[i].x = d * cos(p) * cos(y);
            result.armors[i].y = d * cos(p) * sin(y);
            result.armors[i].z = d * sin(p);
            result.armors[i].yaw = nowstate[2] + PI / 2 * i - atan2(result.y, result.x);
            result.armors[i].status = armormeasure[3] == 1 ? Armor::AVAILABLE : Armor::UNSEEN;
        }

        unlockState();
        return result;
    }

};