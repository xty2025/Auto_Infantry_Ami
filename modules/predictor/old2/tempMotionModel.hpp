#include "interfaceType.hpp"
#include "Eigen/Core"
#include "Log/log.hpp"
#include <vector>
namespace predictor {

class MotionModel 
{
    using VectorY = Eigen::Matrix<double, 4, 1>;
public:
    void initMotionModel()
    {
        init_time = Time::TimeStamp::now();
    };
    Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid)
    {
        Prediction result;
        result.id = carid;
        result.x = fix_dist * cos(last_pyd.yaw) * cos(last_pyd.pitch);
        result.y = fix_dist * sin(last_pyd.yaw) * cos(last_pyd.pitch);
        result.theta = last_pyd.yaw - M_PI;
        result.r1 = 0.3;
        result.r2 = 0.3;
        result.z0 = fix_dist * sin(last_pyd.pitch);
        result.z1 = result.z0;
        for (int i = 0; i < 4; i++)
        {
            result.armors[i].id = i;
            result.armors[i].x = result.x;
            result.armors[i].y = result.y;
            result.armors[i].z = result.z0;
            result.armors[i].yaw = 0;
            result.armors[i].status = Armor::NONEXIST;
        }
        if(first_time)
        {
            return result;
        }
        else
        {
            INFO("T:{}, full_car_T:{}, yaw_min:{}, yaw_max:{}, speed:{}",
                T, T*4, estimate_yaw_min, estimate_yaw_max, 0.25 / T);
            double equalized_time = (timestamp - last_time).toSeconds();
            while(equalized_time > 0)
            {
                equalized_time -= 4 * T;
            }
            equalized_time += (last_time - init_time).toSeconds();
            for(int i=0;i<4;i++)
            {
                //algorithm is to find the two nearest time point
                // and use linear interpolation to get the value
                //firstly find negateive timepoint & positive timepoint
                int neg_index = -1;
                int pos_index = -1;
                for(int j=0;j<armor_measure[i].size();j++)
                {
                    if(armor_measure[i][j].second < equalized_time)
                    {
                        neg_index = j;
                    }
                    else
                    {
                        pos_index = j;
                        break;
                    }
                }
                if((neg_index == -1) || (pos_index == -1))
                {
                    result.armors[i].status = Armor::UNSEEN;
                    continue;
                }
                else 
                {
                    //linear interpolation
                    double neg_yaw = armor_measure[i][neg_index].first.yaw;
                    double pos_yaw = armor_measure[i][pos_index].first.yaw;
                    double neg_pitch = armor_measure[i][neg_index].first.pitch;
                    double pos_pitch = armor_measure[i][pos_index].first.pitch;
                    double neg_time = armor_measure[i][neg_index].second;
                    double pos_time = armor_measure[i][pos_index].second;
                    
                    double yaw = neg_yaw + (pos_yaw - neg_yaw) / (pos_time - neg_time) * (equalized_time - neg_time);
                    double pitch = neg_pitch + (pos_pitch - neg_pitch) / (pos_time - neg_time) * (equalized_time - neg_time);
                    result.armors[i].x = fix_dist * cos(yaw) * cos(pitch);
                    result.armors[i].y = fix_dist * sin(yaw) * cos(pitch);
                    result.armors[i].z = fix_dist * sin(pitch);
                    result.armors[i].yaw = yaw;
                    if(yaw > estimate_yaw_max - shoot_yaw_tol || yaw < estimate_yaw_min + shoot_yaw_tol)
                    {
                        result.armors[i].status = Armor::UNSEEN;
                    }
                    else
                    {
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                }
            }
        }
        return result;
    }
    void Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id)
    {
        if(armor_id < 0 || armor_id > 3)
        {
            WARN("Invalid armor id");
            return;
        }
        if(new_armor[armor_id])
        {
            armor_measure[armor_id].clear();
            new_armor[armor_id] = false;
        }
        PYD measure_pyd = PYD(measure[0], measure[1], measure[2]);
        last_pyd = measure_pyd;
        last_time = timestamp;
        armor_measure[armor_id].push_back({measure_pyd, (timestamp - init_time).toSeconds()});
        for(int i = 0; i < 4; i++)
        {
            if(new_armor[i]) continue;
            if(armor_measure[i].empty()) continue;
            if((timestamp - init_time).toSeconds() > tol_disapper_time + armor_measure[i].back().second)
            {
                new_armor[i] = true;
                double avg_time = 0;
                double min_yaw = std::numeric_limits<double>::max();
                double max_yaw = std::numeric_limits<double>::min();
                for(auto& [pyd, time] : armor_measure[i])
                {
                    avg_time += time;
                    min_yaw = std::min(min_yaw, pyd.yaw);
                    max_yaw = std::max(max_yaw, pyd.yaw);
                }
                avg_time /= armor_measure[i].size();
                center_time.push_back(avg_time);
                yaw_min_ser.push_back(min_yaw);
                yaw_max_ser.push_back(max_yaw);
            }
        }
        if(center_time.size() >= 4)
        {
            if(first_time)
            {
                T = center_time[3] - center_time[2];
                estimate_yaw_max = std::max(yaw_max_ser[2], yaw_max_ser[3]);
                estimate_yaw_min = std::min(yaw_min_ser[2], yaw_min_ser[3]);
                first_time = false;
            }
            else
            {
                T = alpha*(center_time[3] - center_time[2]) + (1-alpha)*T;
                estimate_yaw_max = alpha*yaw_max_ser[3] + (1-alpha)*estimate_yaw_max;
                estimate_yaw_min = alpha*yaw_min_ser[3] + (1-alpha)*estimate_yaw_min;
            }
            center_time.erase(center_time.begin());
            yaw_min_ser.erase(yaw_min_ser.begin());
            yaw_max_ser.erase(yaw_max_ser.begin());
        }
    }

private:
    PYD last_pyd;
    Time::TimeStamp last_time;
    int fix_dist = 5.0;
    int alpha = 0.2;
    Time::TimeStamp init_time;
    bool first_time = true;
    bool new_armor[4] ={true, true, true, true};
    double tol_disapper_time = 0.2;
    std::vector<double> center_time;
    std::vector<double> yaw_min_ser;
    std::vector<double> yaw_max_ser;
    std::vector<std::pair<PYD,double>> armor_measure[4];
    //bool stable = false;
    double estimate_yaw_min = 0;
    double estimate_yaw_max = 0;
    //allow yaw is in the range of [yaw_min+tol, yaw_max-tol]
    double shoot_yaw_tol = 0.0;
    PYD car_center;
    double T = 0.5;
};


}