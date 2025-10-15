#include "MotionModel.hpp"
#include "Log/log.hpp"

using namespace predictor;

void MotionModel::initMotionModel()
{
    MatrixXX P;
    MatrixXX Q;
    MatrixYY R;
    VectorX Q_coef(10);
    Q_coef << 1, 1000, 1, 1000, 1, 1000, 1, 1, 1, 1;
    Q = Q_coef.asDiagonal();
    P.setIdentity();
    Q.setIdentity();
    R.setIdentity();
    P *= 0.1;
    Q *= 0.01;
    R *= 0.1;
    ekf.init(P, Q, R);
}

Prediction MotionModel::getPredictResult(const Time::TimeStamp& timestamp, int carid)
{
    Prediction result;
    auto nowstate = ekf.predict(timestamp);
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
        measure.setId(i);
        VectorY armormeasure;
        measure(nowstate.data(), armormeasure.data());
        double p = armormeasure[0], y = armormeasure[1], d = armormeasure[2];
        result.armors[i].id = i;
        result.armors[i].x = d * cos(p) * cos(y);
        result.armors[i].y = d * cos(p) * sin(y);
        result.armors[i].z = d * sin(p);
        result.armors[i].yaw = nowstate[2] + M_PI / 2 * i - atan2(result.y, result.x);
        // consider float value loss : 1 -> 0.99
        result.armors[i].status = armormeasure[3] >= 0.99 ? Armor::AVAILABLE : Armor::UNSEEN;
    }

    return result;
}

void MotionModel::Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id)
{
    if(firstUpdate)
    {
        firstUpdate = false;
        VectorX first_state = first_state_estimate(measure, armor_id);
        ekf.setX(first_state);
        ekf.setTimeStamp(timestamp);
    }
    else
    {   
        VectorX state = ekf.update(measure, timestamp, armor_id);
        if(state[6]>0.3)state[6]=0.3;
        if(state[7]>0.3)state[7]=0.3;
        ekf.setX(state);
    }
}

VectorX MotionModel::first_state_estimate(const VectorY& measure, const int armor_id)
{
    if(measure[3] == 0) WARN("First state estimate with invisible armor, may cause error");
    VectorX state(10);
    state[0] = (measure[2] * cos(measure[0]) + 0.3) * cos(measure[1]);
    state[1] = (measure[2] * cos(measure[0]) + 0.3) * sin(measure[1]);
    state[2] = measure[1] - M_PI;
    state[3] = 0;
    state[4] = 0;
    state[5] = 0;
    state[6] = 0.3;
    state[7] = 0.3;
    state[8] = -0.05;
    state[9] = -0.05;
    state[8 + armor_id % 2] = measure[2] * sin(measure[0]);
    return state;
}