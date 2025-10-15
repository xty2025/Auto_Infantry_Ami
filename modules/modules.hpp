#pragma once
#include "controller/type.hpp"
#include "driver/type.hpp"
#include "detector/type.hpp"
#include "predictor/type.hpp"
#include "tracker/type.hpp"
#include "solver/type.hpp"
#include "replayer/replayer.hpp"
#include "detector/detector.hpp"
#include "tracker/tracker.hpp"
#include "predictor/predictor.hpp"
#include "solver/solver.hpp"
#include "controller/controller.hpp"
#include "driver/driver.hpp"
//TODO
#include "buff/BuffDetector.hpp"
#include "buff/BuffCalculator.hpp"
#include "buff/BuffController.hpp"
//TODO

namespace modules
{
    using namespace driver;
    using namespace controller;
    using namespace detector;
    using namespace predictor;
    using namespace tracker;
    using namespace solver;
    using namespace replayer;
    using namespace power_rune;

} // namespace modules














// #include "interfaceType.hpp"
// #include <Eigen/Core>

// namespace modules
// {
//     class Driver {
//     public:
//         virtual void setSerialConfig(SerialConfig config) = 0;
//         virtual void setCameraConfig(CameraConfig config) = 0;
//         virtual std::function<void(const ControlResult&)> sendSerialFunc() = 0;
//         virtual void registReadCallback(std::function<void(const ParsedSerialData&)> callback) = 0;
//         virtual void runSerialThread() = 0;
//         virtual void runCameraThread() = 0;
//         virtual bool isExistNewCameraData() = 0;
//         virtual void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) = 0;
//         virtual ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) = 0;
//         virtual void clearSerialData() = 0;
//     };
//     std::unique_ptr<Driver> createDriver();
//     std::unique_ptr<Driver> createReplayer(const std::string& videoPath, const std::string& serialPath);

//     class Controller {//TODO: 开火控制还没写好
//     public:
//         virtual void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc) = 0;
//         //virtual void registSolveFunc(std::function<Projects(const Predictions&, const ParsedSerialData&)> solveFunc) = 0;
//         virtual ControlResult control(const ParsedSerialData& parsedData) = 0;
//     };
//     std::unique_ptr<Controller> createController();

//     class Detector {
//     public:
//         //virtual Detections detect(const ParsedCameraData& frame) = 0;
//         virtual Detections detect(const cv::Mat &image, const cv::Rect &roi = cv::Rect()) = 0;
//         virtual void setEnemyColor(int flag) = 0;//0: red, 1: blue
//     };
//     std::unique_ptr<Detector> createDetector(const std::string &model_path);

//     class Tracker {//TODO: getTrackResult method has HUGE problem for index.
//     public:
//         virtual void merge(const Detections& detections, double threshold=20.0f) = 0;
//         virtual std::vector<cv::Rect2i> calcROI(const XYVs& projects, int width=416, int height=416, int camera_width=1280, int camera_height=1024) = 0;//TODO: change to use ArmorXYVs to auto adjust the roi size.
//         virtual TrackResults getTrackResult(const std::vector<std::tuple<XYV,int,int>> &old_prediction, Time::TimeStamp time) = 0;
//         virtual void registCoordCorrectFunc(std::function<std::pair<double, double>(double, double)> coordCorrectFunc) = 0;
//         virtual bool isDetected() = 0;
//     };
//     std::unique_ptr<Tracker> createTracker();

//     class Solver {//TODO: 传入初始值并改用迭代式算法进行pnp求解可能会增加稳定性。
//     public:
//         virtual XYV world2camera(const PYD& point, const ImuData& imuData) = 0;
//         virtual PYDs camera2world(const ArmorXYVs& trackResults, const ImuData& imuData, bool isLarge) = 0;
//         virtual std::pair<double, double> camera2camera(double center_x, double center_y, double imu_yaw, double imu_pitch) = 0;
//         virtual std::pair<double, double> gen_calcPitchYaw(double gen_center_x, double gen_center_y) = 0;
//         virtual std::function<std::pair<double, double>(double, double)> gen_calcPitchYawFunc() = 0;
//         virtual void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix) = 0;
//         virtual void setCameraOffset(const Eigen::Vector3d& cameraOffset) = 0;
//         virtual void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients) = 0;//k1,k2,p1,p2,k3
//     };
//     std::unique_ptr<Solver> createSolver(double X, double Y, double X1, double Y1, double X2, double Y2);

//     class Predictor {//TODO: 多车同时检测的记忆功能，加快收敛速度。
//     public://TODO: 应当新增自动初始化解算函数  VITAL
//         virtual std::function<Predictions(Time::TimeStamp)> predictFunc() = 0;
//         virtual Predictions predict(Time::TimeStamp timestamp) = 0;
//         virtual void update(const TrackResults& trackResults, const Time::TimeStamp& timestamp) = 0;
//     };
//     std::unique_ptr<Predictor> createPredictor();
// }
