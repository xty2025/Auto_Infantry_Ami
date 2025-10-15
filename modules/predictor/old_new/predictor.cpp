#include "predictor.hpp"
#include "Udpsend/udpsend.hpp"
#include <Log/log.hpp>
#include "Param/param.hpp"

auto predictor::createPredictor() -> std::unique_ptr<Predictor> {
    return std::make_unique<predictor::Predictor>();
}

namespace predictor {
    std::function<Predictions(Time::TimeStamp)> Predictor::predictFunc()
    {
        return std::function<Predictions(Time::TimeStamp)>([&](Time::TimeStamp timestamp) { return predict(timestamp); });
    }
    Predictions Predictor::predict(Time::TimeStamp timestamp)
    {
        Predictions predictions;
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto& [carid, car] : cars)
        {
            if(car->Stable())
            {
                predictions.push_back(model2world(car->getPredictResult(timestamp),std::function<VectorY(const VectorX&, int)>([&](const VectorX& state, int armorid) {
                    return car->measureFromState(state, armorid);
                })));
                predictions.back().id = carid;
                predictions.back().stable = car->armorStable();
            }
            else
                WARN("Car {} is not stable", carid);
        }
        return predictions;
    }
    void Predictor::update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp)
    {
        std::map<int, std::vector<std::tuple<VectorY, int, location::Location, bool>>> measures;
        for(const auto& trackResult : trackResults.first)
        {
            VectorY measure;
            XYZ armor_xyz = trackResult.location.xyz_imu;
            measure[0] = armor_xyz.x;
            measure[1] = armor_xyz.y;
            measure[2] = armor_xyz.z;
            measure[3] = trackResult.yaw;
            measures[trackResult.car_id].push_back(std::make_tuple(measure, trackResult.armor_id, trackResult.location, false));
            //INFO("track: car_id: {}, armor_id: {}, x: {}, y: {}, distance: {}", trackResult.car_id, trackResult.armor_id, measure[0], measure[1], measure[2]);
            detect_count[trackResult.car_id] = 0;
        }
        //INFO("there");
        for(const auto& trackResult : trackResults.second)
        {
            //calc minMaxTheta
            location::Location edge;
            CXYD temp;
            double leftx = trackResult.bounding_rect.x;
            double rightx = trackResult.bounding_rect.x + trackResult.bounding_rect.width;
            if(measures.find(trackResult.car_id) == measures.end())
                continue;//只检测到框没有装甲板，则舍弃
            for(auto& measure_tuple: measures[trackResult.car_id])
            {
                edge.imu = std::get<2>(measure_tuple).imu;
                INFO("debug1 imu: pitch: {}, yaw: {}, distance: {}", edge.imu.pitch, edge.imu.yaw, edge.imu.distance);
                temp = std::get<2>(measure_tuple).cxy;
                temp.cx = leftx;
                edge.cxy = temp;
                std::get<0>(measure_tuple)[4] = static_cast<PYD>(edge.pyd_imu).yaw;
                temp.cx = rightx;
                edge.cxy = temp;
                std::get<0>(measure_tuple)[5] = static_cast<PYD>(edge.pyd_imu).yaw;
                INFO("left-right: {}, {}", std::get<0>(measure_tuple)[5], std::get<0>(measure_tuple)[4]);
                INFO("left-right = {}", std::get<0>(measure_tuple)[5] - std::get<0>(measure_tuple)[4]);
                std::get<3>(measure_tuple) = true;
            }
        }
        //INFO("there1");
        for(auto& [carid, measure_vec] : measures)
        {
            std::lock_guard<std::mutex> lock(car_mutex);
            if(cars.find(carid) == cars.end())
            {
                cars[carid] = std::make_unique<MotionModel>();
                cars[carid]->initMotionModel();
            }
            //一起更新，没有选择某一个装甲板更新
            //先计算出有哪些装甲板id
            if(measure_vec.size() == 1)
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]));
            else if(measure_vec.size() == 2)
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]), std::get<1>(measure_vec[1]));
            else if(measure_vec.size() >= 3)
            {
                ERROR("Impossible measure_vec size: {}", measure_vec.size());
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]), std::get<1>(measure_vec[1]));
            }

            for(auto& measure: measure_vec)
                if(std::get<3>(measure))//确保检测到框
                    cars[carid]->Update(world2model(std::get<0>(measure)), timestamp, std::get<1>(measure));
        }
        //INFO("there2");
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto it = detect_count.begin(); it != detect_count.end();)
        {
            if(it->second > MaxMissFrame)
            {
                cars.erase(it->first);
                it = detect_count.erase(it);
            }
            else
            {
                it->second++;
                ++it;
            }
        }
    }

//state: x,vx,y,vy,theta,omega,r1,r2,z1,z2
//function param: VectorX, int -> VectorY
    Prediction Predictor::model2world(const VectorX& state, std::function<VectorY(const VectorX&, int)> measureFunc)
    {
        INFO("ENTER model2world");
        INFO("ENTER state: {}, {}, {}, {}, {}, {}, {}, {}, {}, {}", state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9]);
        Prediction prediction;
        XYZ xyz;
        xyz.x = -state[2];
        xyz.y = -state[0];
        xyz.z = (state[8] + state[9]) / 2.0;
        prediction.center = xyz;
        prediction.vx = -state[3];
        prediction.vy = -state[1];
        prediction.z1 = state[8];
        prediction.z2 = state[9];
        prediction.theta = state[4];
        prediction.omega = state[5];
        prediction.r1 = state[6];
        prediction.r2 = state[7];
        for (int i = 0; i < 4; i++)
        {

            VectorY armormeasure = measureFunc(state, i);
            XYZ xyz_armor;
            //xyz_armor.x = -armormeasure[1];
            //xyz_armor.y = -armormeasure[0];
            //xyz_armor.z = armormeasure[2];
            double dist = armormeasure[2];
            xyz_armor.z = dist * std::sin(armormeasure[0]);
            xyz_armor.x = -dist * std::cos(armormeasure[0]) * std::sin(armormeasure[1]);
            xyz_armor.y = -dist * std::cos(armormeasure[0]) * std::cos(armormeasure[1]);
            prediction.armors[i].center = xyz_armor;
            prediction.armors[i].yaw = M_PI - armormeasure[3];
            prediction.armors[i].id = i;
            prediction.armors[i].theta = prediction.theta + M_PI / 2 * i;
            double see_angle = std::atan2(state[0],state[2]);
            double armor_angle = std::remainder(prediction.armors[i].theta - M_PI / 2 - see_angle, 2 * M_PI);
            if(armor_angle < M_PI / 6 && armor_angle > -M_PI / 6)
            {
                prediction.armors[i].status = Armor::AVAILABLE;
            }
            else
            {
                INFO("Invalid armor id {}, for angle : {}",i,armor_angle);
                prediction.armors[i].status = Armor::UNSEEN;
            }
        }
        return prediction;
    }
//measure: ax,ay,az,tangent,angle_left,angle_right
    VectorY Predictor::world2model(const VectorY& measure)
    {
        INFO("ENTER world2model");
        VectorY measure_model;
        //measure_model[0] = -measure[1];
        //measure_model[1] = -measure[0];
        //measure_model[2] = measure[2];
        double armor_x = -measure[1];
        double armor_y = -measure[0];
        double z = measure[2];
        measure_model[2] = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
        measure_model[0] = ceres::atan2(z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
        measure_model[1] = ceres::atan2(armor_y, armor_x);

        measure_model[3] = M_PI - measure[3];
        measure_model[4] = -0.5 * M_PI - measure[4];
        measure_model[5] = -0.5 * M_PI - measure[5];
        INFO("ENTER parameter: {}, {}, {}, {}, {}, {}", measure_model[0], measure_model[1], measure_model[2], measure_model[3], measure_model[4], measure_model[5]);
        return measure_model;
    }

} // namespace predictor
