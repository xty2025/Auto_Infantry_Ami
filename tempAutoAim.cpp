#include <iostream>
#include <ranges>
#include <vector>
#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <modules/RecordSolver/recordsolver.hpp>
#include <Param/param.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
#include <mutex>
#include <Udpsend/udpsend.hpp>
using namespace modules;
using namespace aimlog;
std::map<std::string, int> enemyTrans = {
    {"red", 0},
    {"blue", 1},
    {"auto", -1}
};

double center_yaw = 0.0;
double r = 0.27;
double dist = 5.0;
double omega = 180.0;
double pitch0 = 0.0;
double pitch1 = 1.0;
double height = 0.8;
double pitch_offset = 0.0;
double max_theta_tol = 60.0;
double min_theta_tol = -45.0;
double shoot_yaw_tol = 0.5;
double shoot_pitch_tol = 0.5;

double flyTime = 0.23;

double theta =0.0;
int current_armor =0;//0,1
bool clockwise = true;//顺时针
bool prepare_success = false;

Param param("../config.json");
ControlResult findShootPlace(const ParsedSerialData& parsedData)
{
    static Time::TimeStamp lastTime = Time::TimeStamp::now();
    Time::TimeStamp nowTime = Time::TimeStamp::now();
    double dt = (nowTime - lastTime).toSeconds();
    if(clockwise)
        theta += omega * dt;
    else
        theta -= omega * dt;
    if((theta > max_theta_tol)&&clockwise)
    {
        theta -= 120;
        current_armor = 1 - current_armor;
    }
    else if((theta < -max_theta_tol)&&(!clockwise))
    {
        theta += 120;
        current_armor = 1 - current_armor;
    }
    double theta_rad = theta / 180.0 * M_PI;
    double aimyaw = atan2(r * sin(theta_rad), dist - r * cos(theta_rad)) + center_yaw / 180.0 * M_PI;
    ControlResult result;
    result.yaw_setpoint = aimyaw * 180.0 / M_PI;
        result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
    //result.pitch_setpoint = current_armor == 0 ? pitch0 : pitch1;
    result.pitch_setpoint = atan2(height, dist - r * cos(theta_rad)) * 180.0 / M_PI + pitch_offset;
    
    result.valid = true;
    UdpSend::sendData((float)result.yaw_setpoint);
    UdpSend::sendData((float)result.pitch_setpoint);
    UdpSend::sendData((float)parsedData.yaw_now);
    UdpSend::sendData((float)parsedData.pitch_now);
    UdpSend::sendData((float)theta);
    UdpSend::sendData((float)center_yaw);
    UdpSend::sendData((float)pitch0);
    UdpSend::sendData((float)pitch1);
    UdpSend::sendData((float)current_armor);

    UdpSend::sendTail();
    if(abs(result.yaw_setpoint - parsedData.yaw_now) > shoot_yaw_tol || abs(result.pitch_setpoint - parsedData.pitch_now) > shoot_pitch_tol)
    {
        result.shoot_flag = false;
        WARN("SHOOT cancel for tolerance not reach");
        INFO("shoot_yaw_tol:{}",shoot_yaw_tol);
    }
    else if((theta < min_theta_tol) && clockwise)
    {
        result.shoot_flag = false;
        WARN("SHOOT cancel for theta too small when clockwise");
    }
    else if((theta > -min_theta_tol) && (!clockwise))
    {
        result.shoot_flag = false;
        WARN("SHOOT cancel for theta too big when !clockwise");
    }
    else
    {
        //result.shoot_flag = true;
        result.shoot_flag = param["shoot_enable"].Bool();
        std::cout << "SHOOT" << std::endl; 
    }
    //result.shoot_flag = true;
    lastTime = nowTime;
    return result;
}

// 二聚类函数，将pitch值分为两类
std::pair<double, double> kmeansClustering(const std::vector<double>& data) {
    if (data.size() < 2) {
        return {0.0, 1.0}; // 数据不足时返回默认值
    }
    
    // 初始化聚类中心
    double center1 = *std::min_element(data.begin(), data.end());
    double center2 = *std::max_element(data.begin(), data.end());
    
    const int MAX_ITERATIONS = 100;
    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        std::vector<double> cluster1, cluster2;
        
        // 分配数据点到最近的聚类
        for (const double& point : data) {
            if (std::abs(point - center1) < std::abs(point - center2)) {
                cluster1.push_back(point);
            } else {
                cluster2.push_back(point);
            }
        }
        
        // 检查是否有空聚类
        if (cluster1.empty() || cluster2.empty()) {
            continue;
        }
        
        // 更新聚类中心
        double new_center1 = 0.0, new_center2 = 0.0;
        for (const double& p : cluster1) new_center1 += p;
        for (const double& p : cluster2) new_center2 += p;
        
        new_center1 /= cluster1.size();
        new_center2 /= cluster2.size();
        
        // 检查收敛
        if (std::abs(new_center1 - center1) < 0.001 && std::abs(new_center2 - center2) < 0.001) {
            break;
        }
        
        center1 = new_center1;
        center2 = new_center2;
    }
    
    // 确保center1 < center2
    if (center1 > center2) {
        std::swap(center1, center2);
    }
    
    return {center1, center2};
}


std::unique_ptr<modules::Solver> out_solver;
int waitFrame = 500;
double sum_yaw = 0.0;
double middle_yaw_tol = 0.1;
void prepare(PYDs& detections)
{
    static auto aimParam = param["tmp"];
    static bool firstTime = true;
    //static std::vector<double> pitch_values; // 存储所有检测到的pitch值
    //static bool clusters_updated = false; // 标记是否已计算聚类
    //static const int MIN_SAMPLES = 40; // 进行聚类前需要的最小样本数
    if(firstTime)
    {
      r              = aimParam["r"].Double();
      dist           = aimParam["dist"].Double();
      omega          = aimParam["omega"].Double();
      pitch0         = aimParam["pitch0"].Double();
      pitch1         = aimParam["pitch1"].Double();
      height         = aimParam["height"].Double();
      max_theta_tol  = aimParam["max_theta_tol"].Double();
      min_theta_tol  = aimParam["min_theta_tol"].Double();
      shoot_yaw_tol  = aimParam["shoot_yaw_tol"].Double();
      shoot_pitch_tol= aimParam["shoot_pitch_tol"].Double();
      flyTime        = aimParam["flytime"].Double();
      clockwise      = aimParam["clockwise"].Bool();
      middle_yaw_tol = aimParam["middle_yaw_tol"].Double();
      pitch_offset   = aimParam["pitch_offset"].Double();
    firstTime = false;
    }
    bool reach_middle = false;
    static int nowframe = 0;
    static int totaldetect = 0;
    nowframe++;
    //double current_pitch;
    for(auto& detection: detections)
    {
    //std::cout<<detection.yaw<<std::endl;
        detection.yaw *= 180.0 / M_PI;
        detection.pitch *= 180.0 / M_PI;
        sum_yaw += detection.yaw; 
        if(abs(center_yaw - detection.yaw) < middle_yaw_tol)
        {
            //current_pitch = detection.pitch;
            reach_middle = true;
            // 收集pitch值用于聚类
        //pitch_values.push_back(detection.pitch);
        }
        
        totaldetect +=1;
        std::cout<<center_yaw<<detection.yaw<<std::endl;
    }
    if((totaldetect > 0)&&(!prepare_success))
        center_yaw = sum_yaw / totaldetect;
            // 当收集足够的样本后进行聚类
    // if(pitch_values.size() >= MIN_SAMPLES && !clusters_updated) {
    //     auto [p0, p1] = kmeansClustering(pitch_values);
    //     pitch0 = p0 + aimParam["pitch_offset"].Double();
    //     pitch1 = p1 + aimParam["pitch_offset"].Double();
    //     clusters_updated = true;
    // }

    if((nowframe > waitFrame) && reach_middle)
        {
            theta = clockwise? flyTime * omega : -1 * flyTime * omega;
            
                    // 根据最近的装甲板pitch值确定current_armor
        // if (!detections.empty()) {
        //     double current_pitch_ = current_pitch + aimParam["pitch_offset"].Double();
        //     // 判断当前pitch更接近哪一个聚类中心
        //     current_armor = (std::abs(current_pitch_ - pitch0) < std::abs(current_pitch_ - pitch1)) ? 0 : 1;
        // }
            prepare_success = true;
        }
}
cv::Mat rotateImage180_method1(const cv::Mat& src) {
    cv::Mat dst;
    cv::rotate(src, dst, cv::ROTATE_180);
    return dst;
}

int main() {
    param = param[param["car_name"].String()];
    bool udp_enable = param["UDP"]["enable"].Bool();
    if(udp_enable)
        UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());
    else
        UdpSend::disable();
    auto driver = createDriver();
    auto solver = createSolver(param["X"].Double(), param["Y"].Double(), param["X1"].Double(), 
                                param["Y1"].Double(), param["X2"].Double(), param["Y2"].Double());
    auto detector = createDetector(param["model_path"].String());

 
    SerialConfig config{ param["serial_name"].String(), param["baud_rate"].Int() };
    CameraConfig cameraConfig{
        .cameraSN = param["camera_id"].String(), 
        .exposureTime = param["exposure_time"].Double(),
        .gain = param["gain"].Double()
    };
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    //std::vector<std::pair<PYD,Time::TimeStamp>> oldArmorData;
    driver->registReadCallback([control_func = driver->sendSerialFunc()](const ParsedSerialData& parsedData) {
        if(!prepare_success)
        {
            ControlResult result;
            result.yaw_setpoint = parsedData.yaw_now;
            result.pitch_setpoint = 10.0;
            result.shoot_flag = false;
            result.valid = true;
            
            control_func(result);
            return;
        }
        ControlResult result = findShootPlace(parsedData);
        control_func(result);
    });
    driver->runSerialThread();
    driver->runCameraThread();

    Eigen::Matrix3d cameraIntrinsicMatrix;
    auto intrinsicArray = param["camera_intrinsic_matrix"].to<std::vector<std::vector<double>>>();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            cameraIntrinsicMatrix(i, j) = intrinsicArray[i][j];
    solver->setCameraIntrinsicMatrix(cameraIntrinsicMatrix);
    auto cameraOffset = Eigen::Vector3d(0.0, 0.0, 0.0);
    solver->setCameraOffset(cameraOffset);
    auto distorationCoefficientsArray = param["camera_distortion_matrix"].to<std::vector<double>>();
    Eigen::Vector5d distorationCoefficients;
    for (int i = 0; i < 5; ++i)
        distorationCoefficients(i) = distorationCoefficientsArray[i];
    solver->setDistorationCoefficients(distorationCoefficients);

    bool autoEnemy = (enemyTrans[param["enemy_color"].String()] == -1);
    if(!autoEnemy)
        detector->setEnemyColor(enemyTrans[param["enemy_color"].String()]);


    Time::TimeStamp lastDetectTime;
    int receive_enemy_color = 0;
    //out_solver = std::move(solver);
    bool init = true;
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
        frame->image = rotateImage180_method1(frame->image);
        lastDetectTime = frame->timestamp;
        ParsedSerialData imu = driver->findNearestSerialData(frame->timestamp);
        receive_enemy_color = imu.enemy_color;
        if(autoEnemy)
            detector->setEnemyColor(1 - receive_enemy_color);
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
        auto results = solver->camera2world(detectResults,imu,false);
        if(init){
        center_yaw = imu.yaw_now;
        init = false;
        }
        prepare(results);
    }

    return 0;
}