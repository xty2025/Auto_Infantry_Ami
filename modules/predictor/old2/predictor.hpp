#pragma once
#include "interfaceType.hpp"
#include "modules.hpp"
#include "tempMotionModel2.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace predictor {

class Predictor : public modules::Predictor {
private:
    std::mutex car_mutex;
    std::map<int, std::unique_ptr<MotionModel>> cars;
    std::map<int, int> detect_count;//detected then set to 0, not detected then add 1, upper than 10 then delete.
public:
    std::function<Predictions(Time::TimeStamp)> predictFunc() override;
    Predictions predict(Time::TimeStamp timestamp) override;
    void update(const TrackResults& trackResults, const Time::TimeStamp& timestamp) override;
    
};

//A noneed statement only for reminding you.
using modules::createPredictor;

} // namespace predictor
