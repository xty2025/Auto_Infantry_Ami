#include <iostream>
#include <ranges>
#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <modules/RecordSolver/recordsolver.hpp>
#include <Param/param.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
using namespace modules;
using namespace aimlog;

int main() {
    Param param("../config.json");
    param = param["quanxiang"];

    auto driver = createDriver();
    auto solver = createSolver(param["X"].Double(), param["Y"].Double(), param["X1"].Double(), 
                                param["Y1"].Double(), param["X2"].Double(), param["Y2"].Double());
    auto detector = createDetector("../utils/models/armor_yolo_x.xml");


    SerialConfig config{"/dev/ttyACM0", 115200};
    //SerialConfig config{"/dev/pts/11", 115200};
    CameraConfig cameraConfig{.cameraSN = "KE0200060397", .exposureTime = 5000};
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    driver->registReadCallback([control_func = driver->sendSerialFunc() ](const ParsedSerialData& parsedData) {
        ControlResult result = ControlResult();
        result.valid = false;
        control_func(result);
    });
    driver->runSerialThread();
    driver->runCameraThread();

    Eigen::Matrix3d cameraIntrinsicMatrix;
    // "fx": 1268.19886430029,
    //     "fy": 1268.23774017347,
    //     "u0": 630.954427985621,
    //     "v0": 487.698003221433,
    //     "k1": -0.196854458412759,
    //     "k2": 0.0928923269301880,
    //     "p1": 0,
    //     "p2": 0,
    //     "k3": 0.0642790747423870,
    cameraIntrinsicMatrix << 1268.19886430029, 0, 630.954427985621,
                             0, 1268.23774017347, 487.698003221433,
                             0, 0, 1;
    recordSolver::setCameraIntrinsicMatrix(cameraIntrinsicMatrix);
    //auto cameraOffset = Eigen::Vector3d(0.0, 0.0, 0.0);
    //recordSolver::setCameraOffset(cameraOffset);
    Eigen::Vector5d distorationCoefficients = Eigen::Vector5d(-0.196854458412759, 0.0928923269301880, 0, 0, 0.0642790747423870);
    recordSolver::setDistorationCoefficients(distorationCoefficients);

    detector->setEnemyColor(1);

    Time::TimeStamp lastDetectTime;
    while(1)
    {
        if(!driver->isExistNewCameraData())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        lastDetectTime = frame->timestamp;
        ParsedSerialData imu = driver->findNearestSerialData(frame->timestamp);
        driver->clearSerialData();
        auto detections = detector->detect(frame->image);
        ArmorXYVs detectResults;
        for(const auto &detection : detections)
        {
            if(detection.corners.size() != 4)
            {
                WARN("Invalid detection result");
                continue;
            }
            ArmorXYV detectResult;
            for(int i = 0; i < 4; i++)
                detectResult[i] = XYV(detection.corners[i].x, detection.corners[i].y);
            detectResults.push_back(detectResult);
        }
        recordSolver::recordPitchYawData(detectResults, imu);
        recordSolver::recordDistData(detectResults, imu, false);
    }
    return 0;
}