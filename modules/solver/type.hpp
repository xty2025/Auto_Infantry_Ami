#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include <Location/location.hpp>

namespace Eigen {
    // define Eigen5d
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
}
// 前向声明 driver::ParsedSerialData
namespace driver {
    struct ParsedSerialData;
}

namespace solver
{
    struct ImuData
    {
        float pitch;
        float yaw;
        float roll;
        ImuData() {};
        ImuData(const driver::ParsedSerialData& x);
        operator PYD() const
        {
            return PYD(pitch * M_PI / 180, yaw * M_PI / 180, 0);
        };
    };
}