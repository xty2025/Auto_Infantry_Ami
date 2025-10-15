#pragma once
#include "MotionModel.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "TimeStamp/TimeStamp.hpp"
#include "type.hpp"
#include "tracker/type.hpp"

namespace predictor {
    using tracker::TrackResultPairs;

class Predictor{
private:
    std::mutex car_mutex;
    std::map<int, std::unique_ptr<MotionModel>> cars;//每个车的运动模型
    std::map<int, int> detect_count;//detected then set to 0, not detected then add 1, upper than 10 then delete.
    const int MaxMissFrame = 10;
    //超过10帧检测不到就删除
    //世界坐标转预测结果，模型结果转世界坐标
    VectorY world2model(const VectorY& measure);
    Prediction model2world(const VectorX& state, std::function<VectorY(const VectorX&, int)> measureFunc);
public:
    std::function<Predictions(Time::TimeStamp)> predictFunc();//封装了预测逻辑。
    Predictions predict(Time::TimeStamp timestamp);
    //核心预测函数，返回所有预测结果
    void update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp);
    bool Stable() const{return true;};
    
};
std::unique_ptr<Predictor> createPredictor();

} // namespace predictor
