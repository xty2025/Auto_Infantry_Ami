#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "interfaceType.hpp"
#include "Udpsend/udpsend.hpp"

namespace predictor{
    //measure : gen_x, gen_y, distance
    using VectorY = Eigen::Matrix<double, 5, 1>;
    //measure-nano : gen_x, gen_y
    using VectorYN = Eigen::Matrix<double, 2, 1>; // VectorY-nano which discard distance
    using VectorXS = Eigen::Matrix<double, 2, 1>; // VectorX-still
    using VectorXL = Eigen::Matrix<double, 4, 1>; // VectorX-linear
    using VectorXR = Eigen::Matrix<double, 9, 1>; // VectorX-rotate
    enum CarStatus
    {
        STILL,
        LINEAR,
        ROTATE,
        UNSTABLE
    };

    class StillModel
    {
    public:
        struct measureFunc{
            template<typename T>
            void operator()(const T s[2], T m[2]){
                m[0] = s[0];
                m[1] = s[1];
            }
            int id;
            void setId(int id){ this->id = id; }
        };
        struct stateTransFunc{
            template<typename T>
            void operator()(const T& x, T& xp){
                xp[0] = x[0];
                xp[1] = x[1];
            }
            double dt;
            void setDt(double dt){ this->dt = dt; }
        };
        typedef ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, 2, 2> StillEKF;
        std::map<int, StillEKF> ekfs;
        std::map<int, Time::TimeStamp> lastTime;

        //double weight;
        
        Eigen::Matrix<double, 2, 2> P;
        Eigen::Matrix<double, 2, 2> Q;
        Eigen::Matrix<double, 2, 2> R;
        VectorXS last_state = VectorXS::Zero();
        void initStillModel(Eigen::Matrix<double, 2, 2> P, Eigen::Matrix<double, 2, 2> Q, Eigen::Matrix<double, 2, 2> R/*, double weight*/)
        {
            this->P = P;
            this->Q = Q;
            this->R = R;
            //this->weight = weight;
        }
        // void setWeight(double weight){ this->weight = weight; }
        VectorYN getPredictMeasure(const Time::TimeStamp& timestamp, int armorid){return ekfs[armorid].predict(timestamp);}
        bool existArmor(int armorid){ return ekfs.find(armorid) != ekfs.end(); }
        void Reset(int armorid){ ekfs.erase(armorid); }
        Time::TimeStamp CheckTime(int armorid){ return lastTime[armorid]; }
        void RemoveMissArmor(const double time_tol)
        {
            for(auto it = lastTime.begin(); it != lastTime.end();)
            {
                if((Time::TimeStamp::now() - it->second).toSeconds() > time_tol)
                {
                    ekfs.erase(it->first);
                    it = lastTime.erase(it);
                }
                else
                    it++;
            }
        }
        double Update(const VectorYN& measure, const Time::TimeStamp& timestamp, int armor_id) // return error
        {
            if(ekfs.find(armor_id) == ekfs.end())
            {
                VectorXS first_state = VectorXS::Zero();
                first_state[0] = measure[0];
                first_state[1] = measure[1];
                ekfs[armor_id].init(P, Q, R);
                ekfs[armor_id].setTimeStamp(timestamp);
                ekfs[armor_id].setX(first_state);
                lastTime[armor_id] = timestamp;
                return -1;
            }
            else
            {
                lastTime[armor_id] = timestamp;
                VectorYN predict_measure = getPredictMeasure(timestamp, armor_id);
                //UdpSend::sendData((float)predict_measure[0]);
                //UdpSend::sendData((float)predict_measure[1]);
                //UdpSend::sendData((float)measure[0]);
                //UdpSend::sendData((float)measure[1]);

                last_state = ekfs[armor_id].update(measure, timestamp, armor_id);
                return (predict_measure - measure).norm();
            }
        }
    };

    class LinearModel
    {
    public:
        struct measureFunc{
            template<typename T>
            void operator()(const T s[4], T m[2]){
                m[0] = s[0];
                m[1] = s[1];
            }
            int id;
            void setId(int id){ this->id = id; }
        }measureFuncInstance;
        struct stateTransFunc{
            template<typename T>
            void operator()(const T& x, T& xp){
                xp[0] = x[0] + x[2] * dt;
                xp[1] = x[1] + x[3] * dt;
                xp[2] = x[2];
                xp[3] = x[3];
            }
            double dt;
            void setDt(double dt){ this->dt = dt; }
        };
        typedef ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, 4, 2> LinearEKF;
        std::map<int, LinearEKF> ekfs;
        std::map<int, Time::TimeStamp> lastTime;
        
        //double weight;

        Eigen::Matrix<double, 4, 4> P;
        Eigen::Matrix<double, 4, 4> Q;
        Eigen::Matrix<double, 2, 2> R;
        VectorXL last_state = VectorXL::Zero();
        void initLinearModel(Eigen::Matrix<double, 4, 4> P, Eigen::Matrix<double, 4, 4> Q, Eigen::Matrix<double, 2, 2> R/*, double weight*/)
        {
            this->P = P;
            this->Q = Q;
            this->R = R;
            //this->weight = weight;
        }
        bool existArmor(int armorid){ return ekfs.find(armorid) != ekfs.end(); }
        // void setWeight(double weight){ this->weight = weight; }
        VectorYN getPredictMeasure(const Time::TimeStamp& timestamp, int armorid)
        {
            VectorXL state = ekfs[armorid].predict(timestamp);
            VectorYN result;
            measureFuncInstance.setId(armorid);
            measureFuncInstance(state.data(), result.data());
            return result;
        }
        void Reset(int armorid){ ekfs.erase(armorid); }
        Time::TimeStamp CheckTime(int armorid){ return lastTime[armorid]; }
        void RemoveMissArmor(const double time_tol)
        {
            for(auto it = lastTime.begin(); it != lastTime.end();)
            {
                if((Time::TimeStamp::now() - it->second).toSeconds() > time_tol)
                {
                    ekfs.erase(it->first);
                    it = lastTime.erase(it);
                }
                else
                    it++;
            }
        }
        double Update(const VectorYN& measure, const Time::TimeStamp& timestamp, int armor_id) // return error
        {
            if(ekfs.find(armor_id) == ekfs.end())
            {
                VectorXL first_state = VectorXL::Zero();
                first_state = last_state;
                first_state[0] = measure[0];
                first_state[1] = measure[1];
                ekfs[armor_id].init(P, Q, R);
                ekfs[armor_id].setTimeStamp(timestamp);
                ekfs[armor_id].setX(first_state);
                lastTime[armor_id] = timestamp;
                return -1;
            }
            else
            {
                lastTime[armor_id] = timestamp;
                VectorYN predict_measure = getPredictMeasure(timestamp, armor_id);
                last_state = ekfs[armor_id].update(measure, timestamp, armor_id);
                return (predict_measure - measure).norm();
            }
        }
    };

    class RotateModel
    {
    public:
        struct measureFunc{
            template<typename T>
            void operator()(const T s[9], T m[2]){
                auto centerx = s[0];
                auto centery = s[1];
                if(id == 0 || id == 2)centery += s[8];
                auto theta = s[2] + id * M_PI / 2;
                m[0] = centerx - s[7] * sin(theta);
                m[1] = centery - s[6] * cos(theta);
            }
            int id;
            void setId(int id){ this->id = id; }
        }measureFuncInstance;
        struct stateTransFunc{
            template<typename T>
            void operator()(const T& x, T& xp){
                xp[0] = x[0] + x[3] * dt;
                xp[1] = x[1] + x[4] * dt;
                xp[2] = x[2] + x[5] * dt;
                xp[3] = x[3];
                xp[4] = x[4];
                xp[5] = x[5];
                xp[6] = x[6];
                xp[7] = x[7];
                xp[8] = x[8];
            }
            double dt;
            void setDt(double dt){ this->dt = dt; }
        };
        typedef ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, 9, 2> RotateEKF;
        RotateEKF ekf;
        bool first_update = true;

        //double weight;
        
        Eigen::Matrix<double, 9, 9> P;
        Eigen::Matrix<double, 9, 9> Q;
        Eigen::Matrix<double, 2, 2> R;
        VectorXR init_state = VectorXR::Zero();
        void initRotateModel(Eigen::Matrix<double, 9, 9> P, Eigen::Matrix<double, 9, 9> Q, Eigen::Matrix<double, 2, 2> R, VectorXR init_state/*, double weight*/)
        {
            this->P = P;
            this->Q = Q;
            this->R = R;
            this->init_state = init_state;
            //this->weight = weight;
        }
        // void setWeight(double weight){ this->weight = weight; }
        VectorYN getPredictMeasure(const Time::TimeStamp& timestamp, int armorid)
        {
            VectorXR state = ekf.predict(timestamp);
            VectorYN result;
            measureFuncInstance.setId(armorid);
            measureFuncInstance(state.data(), result.data());
            return result;
        }
        bool canSeeArmor(Time::TimeStamp timestamp, int armorid)
        {
            VectorXR state = ekf.predict(timestamp);
            auto theta = state[2];
            auto adjusted_theta = std::fmod(theta + M_PI_2 * armorid, 2 * M_PI);
            if (adjusted_theta < 0)
                adjusted_theta += 2 * M_PI;
            return (adjusted_theta >= M_PI * 0.6 && adjusted_theta <= M_PI * 1.4);
        }
        double Update(const VectorYN& measure, const Time::TimeStamp& timestamp, int armor_id) // return error
        {
            if(first_update)
            {
                ekf.init(P, Q, R);
                ekf.setTimeStamp(timestamp);
                ekf.setX(init_state);
                first_update = false;
                return -1;
            }
            else
            {
                VectorYN predict_measure = getPredictMeasure(timestamp, armor_id);
//                 if(armor_id==0)
// {                UdpSend::sendData((float)predict_measure[0]);
//                 UdpSend::sendData((float)predict_measure[1]);
//                 UdpSend::sendData((float)measure[0]);
//                 UdpSend::sendData((float)measure[1]);}
                ekf.update(measure, timestamp, armor_id);
                return (predict_measure - measure).norm();
            }
        }
    };

    class DistEKF
    {
    public:
        struct measureFunc{
            template<typename T>
            void operator()(const T s[1], T m[1]){
                m[0] = s[0];
            }
            void setId(int){}
        };
        struct stateTransFunc{
            template<typename T>
            void operator()(const T& x, T& xp){
                xp[0] = x[0];
            }
            void setDt(double){}
        };
        ekf::timeEKF::TimeEKF<stateTransFunc, measureFunc, 1, 1> ekf;
        bool first_update = true;
        void initDistEKF(double P, double Q, double R)
        {
            ekf.init(Eigen::Matrix<double, 1, 1>::Identity() * P, Eigen::Matrix<double, 1, 1>::Identity() * Q, Eigen::Matrix<double, 1, 1>::Identity() * R);
        }
        void Update(const double measure, const Time::TimeStamp& timestamp)
        {
            if(first_update)
            {
                ekf.setTimeStamp(timestamp);
                Eigen::Matrix<double, 1, 1> first_state;
                first_state << measure;
                ekf.setX(first_state);
                first_update = false;
            }
            else
            {
                //UdpSend::sendData((float)measure);
                //UdpSend::sendData((float)distPredict());
                ekf.update(Eigen::Matrix<double, 1, 1>::Constant(measure), timestamp, 0);
            }
        }
        double distPredict(void)
        {
            return ekf.getState()[0];
        }
    };

    //算法描述：
    //1. 初始化：初始化三个模型，每个模型有一个权重和一个误差
    //2. 添加模型：如果没有模型，则添加一个模型
    //3. 更新模型：对每个模型进行更新
    //4. 更新权重：根据误差更新权重
    //              误差较小时，先验地信任S > L > R
    //5. 删除模型：删除权重过小的模型 + 删除误差过大(发散)的模型 + 删除长时间未更新的模型Armor
    //6. 分析误差：分析误差和权重，判断是否稳定
    //更新：添加模型->更新模型->更新权重->删除模型->分析误差


    //TODO:rotate 不可见装甲板矫正
    //    旋转模型的初始化状态
    //      旋转模型的三个pushback
    //      getPredictResult
    //      KalmanFilter调参

    class MotionModel 
    {
    public:
        void initMotionModel()
        {
            P_still = Eigen::Matrix<double, 2, 2>::Identity();
            Q_still = Eigen::Matrix<double, 2, 2>::Identity();
            R_still = Eigen::Matrix<double, 2, 2>::Identity();
            P_linear = Eigen::Matrix<double, 4, 4>::Identity();
            Q_linear = Eigen::Matrix<double, 4, 4>::Identity();
            R_linear = Eigen::Matrix<double, 2, 2>::Identity();
            P_rotate = Eigen::Matrix<double, 9, 9>::Identity();
            Q_rotate = Eigen::Matrix<double, 9, 9>::Identity();
            R_rotate = Eigen::Matrix<double, 2, 2>::Identity();
            Eigen::Matrix<double, 9, 1> Q_coef;
            Eigen::Matrix<double, 2, 1> R_coef;
            Eigen::Matrix<double, 9, 1> P_coef;
            Q_coef << 0.05, 0.05, 0.1, 1, 1, 2, 0.05, 0.2, 1;
            Q_rotate = Q_coef.asDiagonal();
            R_coef << 5, 5;
            R_rotate = R_coef.asDiagonal();
            P_coef << 10, 10, 0.1, 100, 100, 1, 1, 1, 0.1;
            P_rotate = P_coef.asDiagonal();

            distEKF.initDistEKF(1, 0.1, 0.1);
        }
        Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid)
        {
            if(status == STILL)
            {
                Prediction result;
                result.id = carid;
                //search weight bigget as the stillModel
                double max_weight = 0.0;
                std::shared_ptr<StillModel> stillModel_ptr;
                for(auto& [stillModel, weight, error, life] : stillModels)
                {
                    if(weight > max_weight)
                    {
                        max_weight = weight;
                        stillModel_ptr = std::make_shared<StillModel>(stillModel);
                    }
                }
                result.dist = distEKF.distPredict();
                for (int i = 0; i < 4; i++)
                {
                    if(stillModel_ptr->existArmor(i))
                    {
                        VectorYN armormeasure = stillModel_ptr->getPredictMeasure(timestamp, i);
                        result.armors[i].id = i;
                        // UdpSend::sendData((float)armormeasure[0]);
                        // UdpSend::sendData((float)armormeasure[1]);

                        result.armors[i].gen_cam_x = armormeasure[0];
                        result.armors[i].gen_cam_y = armormeasure[1];
                        result.armors[i].gen_cam_dist = distEKF.distPredict();
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                    else
                    {
                        result.armors[i].id = i;
                        result.armors[i].status = Armor::UNSEEN;
                    }
                }
                return result;
            }
            else if(status == LINEAR)
            {
                Prediction result;
                result.id = carid;
                //search weight bigget as the linearModel
                double max_weight = 0.0;
                std::shared_ptr<LinearModel> linearModel_ptr;
                for(auto& [linearModel, weight, error, life] : linearModels)
                {
                    if(weight > max_weight)
                    {
                        max_weight = weight;
                        linearModel_ptr = std::make_shared<LinearModel>(linearModel);
                    }
                }
                result.dist = distEKF.distPredict();
                for (int i = 0; i < 4; i++)
                {
                    if(linearModel_ptr->existArmor(i))
                    {
                        VectorYN armormeasure = linearModel_ptr->getPredictMeasure(timestamp, i);
                        result.armors[i].id = i;
                        result.armors[i].gen_cam_x = armormeasure[0];
                        result.armors[i].gen_cam_y = armormeasure[1];
                        result.armors[i].gen_cam_dist = distEKF.distPredict();
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                    else
                    {
                        result.armors[i].id = i;
                        result.armors[i].status = Armor::UNSEEN;
                    }
                }
                return result;
            }
            else if(status == ROTATE)
            {
                Prediction result;
                result.id = carid;
                //search weight bigget as the rotateModel
                double max_weight = 0.0;
                std::shared_ptr<RotateModel> rotateModel_ptr;
                for(auto& [rotateModel, weight, error, life] : rotateModels)
                {
                    if(weight > max_weight)
                    {
                        max_weight = weight;
                        rotateModel_ptr = std::make_shared<RotateModel>(rotateModel);
                    }
                }
                result.dist = distEKF.distPredict();
                for (int i = 0; i < 4; i++)
                {
                    if(rotateModel_ptr->canSeeArmor(timestamp, i))
                    {
                        VectorYN armormeasure = rotateModel_ptr->getPredictMeasure(timestamp, i);
                        result.armors[i].id = i;
                        result.armors[i].gen_cam_x = armormeasure[0];
                        result.armors[i].gen_cam_y = armormeasure[1];
                        result.armors[i].gen_cam_dist = distEKF.distPredict();
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                    else
                    {
                        result.armors[i].id = i;
                        result.armors[i].status = Armor::UNSEEN;
                    }
                }
                return result;
            }
            else
            {
                Prediction result;
                result.id = carid;
                for (int i = 0; i < 4; i++)
                {
                    result.armors[i].id = i;
                    result.armors[i].status = Armor::NONEXIST;
                }
                return result;
            }
        }
        VectorXR first_state_estimate_rotate(std::vector<std::pair<Eigen::VectorXd,int>>& measure_)
        {
            VectorXR state;
            int armor_id = measure_[0].second;
            Eigen::VectorXd measure = measure_[0].first;
            state << measure[0], measure[1] - 50, M_PI - armor_id * M_PI / 2, 0, 0, 0, 50, 200, 0;
            return state;
        }
        void addModels(std::vector<std::pair<Eigen::VectorXd,int>>& measure)
        {
            if(stillModels.size() == 0)
            {
                StillModel stillModel;
                stillModel.initStillModel(P_still, Q_still, R_still);
                stillModels.push_back(std::make_tuple(stillModel, 0.5, 0.0, 0));
            }
            if(linearModels.size() == 0)
            {
                LinearModel linearModel;
                linearModel.initLinearModel(P_linear, Q_linear, R_linear);
                linearModels.push_back(std::make_tuple(linearModel, 0.5, 0.0, 0));
            }
            if(rotateModels.size() == 0)
            {
                RotateModel rotateModel;
                rotateModel.initRotateModel(P_rotate, Q_rotate, R_rotate, first_state_estimate_rotate(measure));
                rotateModels.push_back(std::make_tuple(rotateModel, 0.3, 0.0, 0));
            }
        }
        void updateModels(std::vector<std::pair<Eigen::VectorXd,int>>& measure, const Time::TimeStamp& timestamp)
        {
            double temp_error = 0.0;
            int count = 0;
            for(auto& [stillModel, weight, error,life] : stillModels)
            {
                count = 0;
                error = 0.0;
                for(auto& [measure, armorid] : measure)
                {
                    VectorYN measure_;
                    measure_[0] = measure[0];
                    measure_[1] = measure[1];
                    temp_error = stillModel.Update(measure_, timestamp, armorid);
                    error += temp_error >= 0.0 ? temp_error : switchArmorError;
                    count++;
                }
                if(count > 0)
                    error /= count;
            }
            for(auto& [linearModel, weight, error,life] : linearModels)
            {
                count = 0;
                error = 0.0;
                for(auto& [measure, armorid] : measure)
                {
                    VectorYN measure_;
                    measure_[0] = measure[0];
                    measure_[1] = measure[1];
                    temp_error = linearModel.Update(measure_, timestamp, armorid);
                    error += temp_error >= 0.0 ? temp_error : switchArmorError;
                    count++;
                }
                if(count > 0)
                    error /= count;
            }
            for(auto& [rotateModel, weight, error,life] : rotateModels)
            {
                count = 0;
                error = 0.0;
                for(auto& [measure, armorid] : measure)
                {
                    VectorYN measure_;
                    measure_[0] = measure[0];
                    measure_[1] = measure[1];
                    temp_error = rotateModel.Update(measure_, timestamp, armorid);
                    error += temp_error >= 0.0 ? temp_error : switchArmorError;
                    count++;
                }
                if(count > 0)
                    error /= count;
            }
        }
        void updateWeights()
        {
            double total_weight = 0.0;
            double total_error = 0.0;
            int model_count = 0;

            // 计算所有模型的总权重和总误差
            for (const auto& [stillModel, weight, error, life] : stillModels) {
                total_weight += weight;
                total_error += error * weight;
                //UdpSend::sendData((float)error);
                model_count++;
            }
            for (const auto& [linearModel, weight, error, life] : linearModels) {
                total_weight += weight;
                total_error += error * weight;
                //UdpSend::sendData((float)error);
                model_count++;
            }
            for (const auto& [rotateModel, weight, error, life] : rotateModels) {
                total_weight += weight;
                total_error += error * weight;
                //UdpSend::sendData((float)error);
                model_count++;
            }

            // 计算平均误差
            double average_error = total_error / total_weight;

            // 更新 StillModel 的权重
            for (auto& [stillModel, weight, error, life] : stillModels) {
                double relative_error = error / average_error;
                double likelihood = exp(-0.4 * relative_error * relative_error); // 计算似然度
                weight *= likelihood; // 更新权重
            }

            // 更新 LinearModel 的权重
            for (auto& [linearModel, weight, error, life] : linearModels) {
                double relative_error = error / average_error;
                double likelihood = exp(-0.5 * relative_error * relative_error); // 计算似然度
                weight *= likelihood; // 更新权重
            }

            // 更新 RotateModel 的权重
            for (auto& [rotateModel, weight, error, life] : rotateModels) {
                double relative_error = error / average_error;
                double likelihood = exp(-0.6 * relative_error * relative_error); // 计算似然度
                weight *= likelihood; // 更新权重
            }

            // 归一化权重，使总权重为 1
            total_weight = 0.0;
            for (const auto& [stillModel, weight, error, life] : stillModels) {
                total_weight += weight;
            }
            for (const auto& [linearModel, weight, error, life] : linearModels) {
                total_weight += weight;
            }
            for (const auto& [rotateModel, weight, error, life] : rotateModels) {
                total_weight += weight;
            }

            for (auto& [stillModel, weight, error, life] : stillModels) {
                weight /= total_weight;
                //UdpSend::sendData((float)weight);
            }
            for (auto& [linearModel, weight, error, life] : linearModels) {
                weight /= total_weight;
                //UdpSend::sendData((float)weight);
            }
            for (auto& [rotateModel, weight, error, life] : rotateModels) {
                weight /= total_weight;
                //UdpSend::sendData((float)weight);
            }
        }
        void removeModels()
        {
            for(auto it = stillModels.begin(); it != stillModels.end();)
            {
                std::get<0>(*it).RemoveMissArmor(missTime);
                if(std::get<2>(*it) > removeError || std::get<1>(*it) < weight_min)
                {
                    std::get<3>(*it)++;
                    if(std::get<3>(*it) > liveFrame)
                    {
                        it = stillModels.erase(it);
                        continue;
                    }
                }
                it++;
            }
            for(auto it = linearModels.begin(); it != linearModels.end();)
            {
                std::get<0>(*it).RemoveMissArmor(missTime);
                if(std::get<2>(*it) > removeError || std::get<1>(*it) < weight_min)
                {
                    std::get<3>(*it)++;
                    if(std::get<3>(*it) > liveFrame)
                    {
                        it = linearModels.erase(it);
                        continue;
                    }
                }
                it++;
            }
            for(auto it = rotateModels.begin(); it != rotateModels.end();)
            {
                if(std::get<2>(*it) > removeError || std::get<1>(*it) < weight_min)
                {
                    std::get<3>(*it)++;
                    if(std::get<3>(*it) > liveFrame)
                    {
                        it = rotateModels.erase(it);
                        continue;
                    }
                }
                it++;
            }
        }
        void analyseError()
        {
            //根据误差和权重判断当前状态
            double weight_still = 0.0;
            double weight_linear = 0.0;
            double weight_rotate = 0.0;
            double error_still = 0.0;
            double error_linear = 0.0;
            double error_rotate = 0.0;
            for(auto& [stillModel, weight, error, life] : stillModels)
            {
                weight_still += weight;
                error_still += error * weight;
            }
            for(auto& [linearModel, weight, error, life] : linearModels)
            {
                weight_linear += weight;
                error_linear += error * weight;
            }
            for(auto& [rotateModel, weight, error, life] : rotateModels)
            {
                weight_rotate += weight;
                error_rotate += error * weight;
            }
            if(weight_still > weight_statusRecognize_max && error_still < error_statusRecognize_min)
                status = STILL;
            else if(weight_linear > weight_statusRecognize_max && error_linear < error_statusRecognize_min)
                status = LINEAR;
            else if(weight_rotate > weight_statusRecognize_max && error_rotate < error_statusRecognize_min)
                status = ROTATE;
            else
                status = UNSTABLE;

        }
        void Update(std::vector<std::pair<Eigen::VectorXd,int>>& measure, const Time::TimeStamp& timestamp)
        {
            for(int i=0;i<4;i++)
            {
                //if exist in measure output it
                //else output 0
                bool exist = false;
                for(auto& [measure, armorid] : measure)
                {
                    if(armorid == i)
                    {
                        exist = true;
                        break;
                    }
                }
                if(!exist)
                {
                    UdpSend::sendData((float)0.0);
                    UdpSend::sendData((float)0.0);
                    UdpSend::sendData((float)0.0);
                    UdpSend::sendData((float)0.0);
                    UdpSend::sendData((float)0.0);
                }
                else{
                    for(auto& [measure, armorid] : measure)
                    {
                        if(armorid == i)
                        {
                            UdpSend::sendData((float)measure[0]);
                            UdpSend::sendData((float)measure[1]);
                            UdpSend::sendData((float)measure[2]);
                            UdpSend::sendData((float)measure[3]);
                            UdpSend::sendData((float)measure[4]);
                        }
                    }
                }
            }
            // last_armor_measure = measure;
            // //calc alive armor
            // std::vector<int> alive_armor_;
            // for(auto& [measure, armorid] : measure)
            // {
            //     alive_armor_.push_back(armorid);
            // }
            // if(!alive_armor_.empty())
            //     alive_armor = alive_armor_;
            distEKF.Update(measure[0].first[2], timestamp);
            //calc average_id
            // double average_id = 0;
            // for(auto& [measure, armorid] : measure)
            // {
            //     average_id += armorid;
            // }
            // average_id /= measure.size();
            // UdpSend::sendData((float)average_id);
            // UdpSend::sendData((float)measure.size());
            addModels(measure);
            updateModels(measure, timestamp);
            updateWeights();
            analyseError();
            removeModels();
        }
        bool Stable(){ return status != UNSTABLE; }
    private:
        const double switchArmorError = 50;
        const double removeError = 300;
        //const double tolError = 20;
        const double missTime = 0.5;
        const double weight_min = 0.1;
        const double weight_statusRecognize_max = 0.5;
        const double error_statusRecognize_min = 10;
        const int liveFrame = 200;

        //Kalman PQR
        Eigen::Matrix<double, 2, 2> P_still;
        Eigen::Matrix<double, 2, 2> Q_still;
        Eigen::Matrix<double, 2, 2> R_still;
        Eigen::Matrix<double, 4, 4> P_linear;
        Eigen::Matrix<double, 4, 4> Q_linear;
        Eigen::Matrix<double, 2, 2> R_linear;
        Eigen::Matrix<double, 9, 9> P_rotate;
        Eigen::Matrix<double, 9, 9> Q_rotate;
        Eigen::Matrix<double, 2, 2> R_rotate;

        //Model : ModelType, weight, error, life
        std::list<std::tuple<StillModel,double,double,int>> stillModels;
        std::list<std::tuple<LinearModel,double,double,int>> linearModels;
        std::list<std::tuple<RotateModel,double,double,int>> rotateModels;
        DistEKF distEKF;

        // std::vector<std::pair<Eigen::VectorXd,int>> last_armor_measure;
        // std::vector<int> alive_armor;
        CarStatus status = UNSTABLE;
    };
}