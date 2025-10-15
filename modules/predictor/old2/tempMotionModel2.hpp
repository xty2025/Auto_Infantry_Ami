#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "interfaceType.hpp"
#include <Log/log.hpp>

namespace predictor{
    const int N_Y = 3;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

    class MotionModel 
    {
    public:
        void initMotionModel();
        Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid);
        void Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id)
        {
        }
        bool Stable();
    private:
        enum EnemyStatus
        {
            STILL,
            LINERA,
            CIRCLE
        };
        bool stable = false;
        const int begin_size = 10;
        EnemyStatus status = STILL;
        bool firstUpdate = true;
    };
}