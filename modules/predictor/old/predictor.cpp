#include "predictor.hpp"

auto modules::createPredictor() -> std::unique_ptr<modules::Predictor> {
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
            predictions.push_back(car->getPredictResult(timestamp, carid));
        }
        return predictions;
    }
    void Predictor::update(const TrackResults& trackResults, const Time::TimeStamp& timestamp)
    {
        for(const auto& trackResult : trackResults)
        {
            if(cars.find(trackResult.car_id) == cars.end())
            {
                std::lock_guard<std::mutex> lock(car_mutex);
                cars[trackResult.car_id] = std::make_unique<MotionModel>();
                cars[trackResult.car_id]->initMotionModel();
            }
            Dvector measure(4);
            measure[0] = trackResult.location.pitch;
            measure[1] = trackResult.location.yaw;
            measure[2] = trackResult.location.distance;
            measure[3] = trackResult.visible;
            cars[trackResult.car_id]->Update(measure, timestamp, trackResult.armor_id);
            detect_count[trackResult.car_id] = 0;
        }
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto it = detect_count.begin(); it != detect_count.end();)
        {
            if(it->second > 10)
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

} // namespace predictor
