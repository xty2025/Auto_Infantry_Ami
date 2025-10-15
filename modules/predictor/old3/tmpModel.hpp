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
    };
    Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid)
    {
    }
    void Update(const VectorY& measure, const Time::TimeStamp& timestamp, int armor_id)
    {
    }
};


}