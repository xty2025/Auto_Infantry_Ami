#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include "Location/location.hpp"

namespace predictor
{
    struct Armor {
        enum armor_status {
            NONEXIST,//temply not used
            UNSEEN,
            AVAILABLE
        };
        XYZ center;
        double yaw;
        double theta;// like yaw
        int id;
        armor_status status = NONEXIST;
    };

    struct Prediction {
        XYZ center;
        int id;
        double vx, vy;
        double z1, z2;
        double theta, omega;
        double r1, r2;
        std::array<Armor, 4> armors;//4个角点。
        bool stable = true;//检测车体是否稳定的判断
    };

    typedef std::vector<Prediction> Predictions;
};