#include "controller.hpp"
#include <cmath>
#include <Log/log.hpp>
#include <Udpsend/udpsend.hpp>

/*
- 1.
多层次容错机制 ：

- 首选特定装甲板
- 备选最近装甲板
- 最后退化到车体中心
- 2.
物理模型补偿 ：

- 重力补偿确保远距离命中精度
- 飞行时间限制保证系统响应性
- 3.
特殊模式支持 ：

- 陀螺自瞄模式的特殊处理
- 鼠标模式的坐标转换
- 4.
精确射击控制 ：

- 双方向角度偏差检查
- 精确的射击时机判断
*/

auto controller::createController(param::Param json_param) -> std::shared_ptr<controller::Controller>
{
    return std::make_unique<controller::Controller>(json_param);
}


using namespace controller;
using namespace aimlog;
using param::Param;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}
void Controller::readJsonParam()
{
    mouse_require = json_param["mouse_require"].Bool();
    pic_camera_x = json_param["pic_camera_x"].Double();
    pic_camera_y = json_param["pic_camera_y"].Double();
    response_speed = json_param["response_speed"].Double();
    shootDelay = std::chrono::duration<double>(json_param["shoot_delay"].Double());
}

static bool thetaInRange(double theta_deg, double range_deg)
{
    theta_deg = std::remainder(theta_deg, 360.0);
    if(std::abs(theta_deg) < range_deg)
        return true;
    else
        return false;
} 

bool Controller::judgeAimNew(bool request)
{
    aim_new = false;
    if(((!aiming) && request) || (aiming && (!request)))
    {
        accumulate_aim_request++;
    }
    else
    {
        accumulate_aim_request = 0;
    }
    if(accumulate_aim_request > waitFrame)
    {
        accumulate_aim_request = 0;
        aiming = request;
        if(aiming)
            aim_new = true;
    }
    return aim_new;
}

//ControlResult control(const ParsedSerialData& parsedData);
ControlResult Controller::control(const ParsedSerialData& parsedData)
{
    if(judgeAimNew(parsedData.aim_request))
    {
        aim_armor_id = std::make_pair(-1, -1);
    }
    if(parsedData.actual_bullet_speed > min_bullet_speed)
    {
    INFO("parsedData.actual_bullet_speed:{}",parsedData.actual_bullet_speed);
    //弹速的解算：
    
    if(bullet_speed == 0)
    {
        bullet_speed = parsedData.actual_bullet_speed;
    }
    else
    {
        bullet_speed = parsedData.actual_bullet_speed * bullet_speed_alpha + (1 - bullet_speed_alpha) * bullet_speed;
    }

    bool aim_center_request = parsedData.aim_request == 3;


    ControlResult result;
    //为什么放在里面？创建之后留返回的接口
    result.yaw_setpoint = parsedData.yaw_now;
    result.pitch_setpoint = parsedData.pitch_now;
    result.pitch_actual_want = parsedData.pitch_now;
    result.yaw_actual_want = parsedData.yaw_now;
    result.valid = false;
    result.shoot_flag = false;

    Predictions predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime);
    if (predictions_for_time.empty())
    {
        WARN("No prediction");
        return result;
    }
    //找车：如果没有就找相机最近的一个车。
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    //std::any_of() 是C++ STL算法库中的一个函数，用于检查范围内的元[&](const auto& prediction) { return prediction.id == aim_armor_id.first; }: 这是一个lambda表达式（匿名函数）素是否满足特定条件
    //[&](const auto& prediction) { return prediction.id == aim_armor_id.first; } :
    // 这是一个lambda表达式（匿名函数），作为判断条件
    //predictions_for_time返回的指标是预测数据的迭代器。代表返回vector<it.begin(),it.end()>
    if (!is_valid_car_id)
    {
        WARN("Invalid car id");
        //重新选择一个相机距离最近的
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        location::Location tmp;
        tmp.imu = ImuData(parsedData);
        //调用param中的imu数据 
        for (const auto& prediction : predictions_for_time)
        {
            double distance;
            if(!mouse_require)
                distance = prediction.center.x * prediction.center.x + prediction.center.y * prediction.center.y;
            else
            //在鼠标模式下，需要根据相机坐标和鼠标坐标计算距离
            {
                tmp.xyz_imu = prediction.center;
                CXYD tmp_cxy = tmp.cxy;
                distance = (tmp_cxy.cx - pic_camera_x) * (tmp_cxy.cx - pic_camera_x) + (tmp_cxy.cy - pic_camera_y) * (tmp_cxy.cy - pic_camera_y);
            }
            if (distance < min_distance)
            {
                min_distance = distance;
                aim_armor_id.first = prediction.id;
                aim_armor_id.second = -1;
                found = true;
            }
        }
        if(!found)
        {
            WARN("No available car");
            return result;
        }
    }
    //屎山：重复了
    auto it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        WARN("New car id invalid(shouldn't happen)");
        return result;
    }
    //
    //重新查找当前目标在预测中的位置，目标丢失直接返回。
    //calc new flytime。
    double distance = sqrt(it->center.x * it->center.x + it->center.y * it->center.y);
    if (distance > 0.0)
    {
        flyTime = std::chrono::duration<double>(distance / bullet_speed);//粗略计算
        predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime);
    }
    it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        WARN("New car id invalid after flytime update");
        return result;
    }
    bool is_valid_armor_id = std::any_of(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
    if(is_valid_armor_id)
    {
        //计算临界角
        double radius = (aim_armor_id.second % 2 == 0)?it->r1:it->r2;
        //偶数ID用r1，奇数用r2当半径。
        double jump_angle = M_PI / 4 - ((std::abs(it->omega)>0.01)?(1/(1+1.414*distance*response_speed/radius/std::abs(it->omega))):(0));
        //设置动态补偿项，防止角度 too small
        //- 目标距离越远，需要的提前角越大
        //- 旋转速度越快，需要的提前角越小
        //- 半径越大，需要的提前角越小
        //- 响应速度越快，需要的提前角越大
        INFO("BADANGLE:{}",jump_angle);
        if(jump_angle<0)jump_angle = M_PI / 6;
        /*装甲板切换逻辑:
        1. 1.
        顺时针旋转 (omega>0)且 当前装甲板偏航角超过临界角 时：
        - 切换到下一个装甲板： (aim_armor_id.second + 3) % 4
        - 例如：0→3, 1→0, 2→1, 3→2
        2. 2.
        逆时针旋转 (omega<0)且 当前装甲板偏航角低于负临界角 时：
        - 切换到上一个装甲板： (aim_armor_id.second + 1) % 4
        - 例如：0→1, 1→2, 2→3, 3→0*/
        if(std::abs(it->omega)>0.6)//ANTI ROTATE。
        {
            if(it->omega>0&&it->armors[aim_armor_id.second].yaw>jump_angle)
            {
                aim_armor_id.second = (aim_armor_id.second + 3) % 4;
            }
            else if(it->omega<0&&it->armors[aim_armor_id.second].yaw<-jump_angle)
            {
                aim_armor_id.second = (aim_armor_id.second + 1) % 4;
            }
            is_valid_armor_id = std::any_of(it->armors.begin(), it->armors.end(),
                                                 [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
        }
        //重新检查新切换的装甲板是否有效和可用,用到了顺逆
    }
    bool found = false;
    if (!is_valid_armor_id) {
        WARN("Invalid armor id");
        INFO("Invalid armor :{}",aim_armor_id.second);
        if(aim_armor_id.second >= 0)
            INFO("Invalid for yaw:{}",it->armors[aim_armor_id.second].yaw);
        //重新选择一个距离最近的
        double min_distance = std::numeric_limits<double>::max();
        location::Location tmp;
        tmp.imu = ImuData(parsedData);
        for (const auto &armor: it->armors) {
            if (armor.status == Armor::AVAILABLE) {
                double distance;
                if (!mouse_require)
                    distance = armor.center.x * armor.center.x + armor.center.y * armor.center.y;
                else {
                    tmp.xyz_imu = armor.center;
                    CXYD tmp_cxy = tmp.cxy;
                    distance = (tmp_cxy.cx - pic_camera_x) * (tmp_cxy.cx - pic_camera_x) +
                               (tmp_cxy.cy - pic_camera_y) * (tmp_cxy.cy - pic_camera_y);
                }
                if (distance < min_distance) {
                    min_distance = distance;
                    aim_armor_id.second = armor.id;
                    found = true;
                }
            }
        }
    }
    
    //如果没有找到可用的装甲板，或者目标丢失，直接返回。
    if (((!is_valid_armor_id)&&(!found)) || aim_center_request)
    {
        WARN("No available armor");
        WARN("Now choose to aim at the car");
        double pitch = 0.0;
        double yaw = 0.0;
        double time = 0.0;
        bool success = calcPitchYaw(pitch, yaw, time, it->center.x, it->center.y, it->center.z);
        if (!success)
        {
            WARN("calcPitchYaw failed");
            return result;
        }
        result.pitch_setpoint = pitch * 180 / PI;
        result.yaw_setpoint = yaw * 180 / PI;
        result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
        result.pitch_actual_want = result.pitch_setpoint;
        result.yaw_actual_want = result.yaw_setpoint;
        result.valid = true;
        result.shoot_flag = false;
        flyTime = std::chrono::duration<double>(time);
        INFO("aim_pitch:{},aim_yaw:{}", result.pitch_setpoint, result.yaw_setpoint);
    }
    if(aim_center_request)
    {
        predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime + shootDelay);
        it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
                          [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
        if (it == predictions_for_time.end())
        {
            WARN("New car id invalid after flytime update");
            return result;
        }
        for(int i=0;i<4;i++)
        {
            double pitch,yaw,time;
            if(it->armors[i].status == Armor::AVAILABLE)
            {
                auto armor_it = it->armors[i];
                if (!calcPitchYaw(pitch, yaw, time, armor_it.center.x, armor_it.center.y, armor_it.center.z))
                {
                    WARN("calcPitchYaw failed");
                    continue;
                }
            }
            //判断是否是可以射击的标志
            double delta_yaw = std::remainder(pitch - parsedData.pitch_now * M_PI / 180 ,2 * M_PI);
            double delta_pitch = std::remainder(yaw - parsedData.yaw_now * M_PI / 180 ,2 * M_PI);
            double dist = it->armors[i].center.dist();
            if(std::tan(delta_yaw)*dist<tol_deltax && std::tan(delta_pitch)*dist<tol_deltay)
            {
                result.shoot_flag = true;
                return result;
            }//是否在阈值内
        }
        result.shoot_flag = false;
        return result;
    }
    /*射击判断逻辑 ：
1. 1.
   计算角度偏差（使用 std::remainder 处理角度周期性）
2. 2.
   将角度偏差转换为距离偏差： tan(角度差) * 距离
3. 3.
   如果两个方向的距离偏差都小于容忍阈值，则允许射击
4. 4.
   一旦找到可射击的装甲板就立即返回*/
    auto armor_it = std::find_if(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if (armor_it == it->armors.end())
    {
        WARN("Invalid armor id (shoudln't happen)");
        return result;
    }
    //should calc new time
    //but now we just use the old time
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
    if (!calcPitchYaw(pitch, yaw, time, armor_it->center.x, armor_it->center.y, armor_it->center.z))
    {
        WARN("calcPitchYaw failed");
        return result;
    }
    result.pitch_setpoint = pitch * 180 / PI;
    result.yaw_setpoint = yaw * 180 / PI;
    result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
    result.valid = true;

    double delta_yaw = std::remainder(pitch - parsedData.pitch_now * M_PI / 180 ,2 * M_PI);
    double delta_pitch = std::remainder(yaw - parsedData.yaw_now * M_PI / 180 ,2 * M_PI);
    double dist = armor_it->center.dist();
    if(std::tan(delta_yaw)*dist<tol_deltax && std::tan(delta_pitch)*dist<tol_deltay)
    {
        result.shoot_flag = true;
    }
    else
        result.shoot_flag = false;
    INFO("flyTime:{}",flyTime.count());
    return result;
    }
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z){
// Param param("../config.json");

//     param = param[param["car_name"].String()];
//     double twohigh = param["2m_highoffset"].Double();
//     double fivehigh = param["5m_highoffset"].Double();
//     double finalhigh = (std::sqrt(target_x * target_x + target_y * target_y) - 2.0)/3.0 * (fivehigh - twohigh) + twohigh;
//     target_z += finalhigh;
    INFO("targetx,y,z:{},{},{}",target_x,target_y,target_z);
    double distance = sqrt(target_x * target_x + target_y * target_y);
    //double theta = atan2(target_z, distance);
    double theta = pitch;
    double delta_z = 0.0;
    // 首先计算空气阻力系数 K
    double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
    for (int i = 0; i < max_iter; i++)
    {
        // 计算炮弹的飞行时间
        double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));

        delta_z = target_z - bullet_speed * sin(theta) * t / cos(theta) + 0.5 * GRAVITY * t * t / cos(theta) / cos(theta);
        // DLOG(INFO) << "delta_y: " << delta_y;

        // 不断更新theta，直到小于某一个阈值
        if (fabs(delta_z) < tol)
        {
            time = t;
            break;
        }

        // 更新角度
        theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
    }
    if(fabs(delta_z) > tol)
    {
        //不更新pitch和yaw
        WARN("calcPitchYaw failed");
        return false;//计算失败
    }
    else
    {
        pitch = theta;
        yaw = atan2(target_y, target_x);
        return true;
    }
} // namespace controller
