#pragma once
#include <cstdint>

namespace controller
{
    struct ControlResult
    {
        uint8_t shoot_flag;//0,1,2
        float pitch_setpoint;//
        float yaw_setpoint;//定义的拟合发射点的中心
        float pitch_actual_want;
        float yaw_actual_want;//相机返回值的坐标中心
        bool valid=true;

        
    };
}