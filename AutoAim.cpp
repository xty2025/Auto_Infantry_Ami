#include <iostream>
#include <ranges>
#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <Udpsend/udpsend.hpp>
#include <Param/param.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
#include <VideoStreamer/VideoStreamer.hpp>
#include <Recorder/recorder.hpp>
#include <Location/location.hpp>
using namespace modules;
using namespace aimlog;
using namespace recording;

//TODO:
//1.完善选板逻辑 controller
//2.加入整车预测 predictor
//3.日志完善
//4. FATAL 大小装甲板
//5. detector ROI 变换检查

// 控制指令最好不要以超高频率发送
// 控制指令间隔不能较长，须保证串口处于激活状态
// 对多线程共享变量的赋值不能极高频率，否则可能导致另一个线程无法访问。

std::map<std::string, int> enemyTrans = {
    {"red", 0},
    {"blue", 1},
    {"auto", -1},
    {"all", -2},
    {"r&b", -3}
};

int main() {
    param::Param param("../config.json");
    param = param[param["car_name"].String()];
    bool udp_enable = param["UDP"]["enable"].Bool();
    bool web_debug_enable = param["web_debug"].Bool();
    bool shoot_enable = param["shoot_enable"].Bool();
    bool record_enable = param["record_enable"].Bool();
    bool force_shoot = param["force_shoot"].Bool();
    bool draw_debug_image = param["debug_on_image"].Bool();
    /*bool udp_enable = param["UDP"]["enable"].Bool();: 从配置中读取 UDP 字段下的 enable 键，并将其布尔值赋给 udp_enable 变量。这决定了是否启用 UDP 通信。

bool web_debug_enable = param["web_debug"].Bool();: 读取 web_debug 键的布尔值，决定是否启用网页调试功能。

bool shoot_enable = param["shoot_enable"].Bool();: 读取 shoot_enable 键的布尔值，决定是否允许射击。

bool record_enable = param["record_enable"].Bool();: 读取 record_enable 键的布尔值，决定是否启用录像功能。

bool force_shoot = param["force_shoot"].Bool();: 读取 force_shoot 键的布尔值，决定是否强制射击（即使没有瞄准目标）。

bool draw_debug_image = param["debug_on_image"].Bool();: 读取 debug_on_image 键的布尔值，决定是否在图像上绘制调试信息。*/
    if(udp_enable)
        UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());
    else
        UdpSend::disable();
    if(web_debug_enable)
        VideoStreamer::init();
    if(record_enable)
    {
        Recorder::instance().start();
        INFO("Recording started");
    }
    auto driver = createDriver();
    //auto driver = createReplayer("../record/test.mkv", "../record/20250528144828.txt", true, 1);
    //auto solver = createSolver(param["X"].Double(), param["Y"].Double(), param["X1"].Double(),
    //                            param["Y1"].Double(), param["X2"].Double(), param["Y2"].Double());
    auto solver = createSolver(param["solver"]);
    auto controller = createController(param["controller"]);
    auto predictor = createPredictor();
    //auto detector = createDetector(param["model_path"].String(), param["car_model_path"].String(), false);
    auto detector = createDetector(param["detector"], false);
    auto tracker = createTracker();

    //xjj
    std::string red_buff_model_path = (param["buff"])["red_buff_model_path"].String();
    std::string blue_buff_model_path = (param["buff"])["blue_buff_model_path"].String();    
    bool EfficiencyFirst = param["buff"]["EfficiencyFirst"].Bool();// 是否性能优先,可给步兵打大符的优先级。
    BuffDetector buff_detector(red_buff_model_path, blue_buff_model_path); // 创建大能量机关检测器
    BuffCalculator buff_calculator(param);   //属于solver 越級了, 后面加新模塊buff再移
    BuffController buff_controller;
    bool hitBuff = false;// 是否正在攻击大能量机关
    int buff_mode = 0;// 大能量机关模式
    bool buff_success = false;// 大能量机关计算是否成功
    bool reload_big_buff = true;// 是否重新加载大能量机关
    // 新增共享变量

// 新增共享变量，用于在不同线程间传递大能量机关的瞄准角度
    std::shared_ptr<float> buff_pitch = std::make_shared<float>(0.0);
    std::shared_ptr<float> buff_yaw = std::make_shared<float>(0.0);
    

    //xjj
  // 将姿态解算器注册到location模块，用于坐标系转换
    location::Location::registerSolver(solver);
    ParsedSerialData imu;

// 将预测器的预测函数注册到控制器中，使控制器可以使用预测结果
    controller->registPredictFunc(predictor->predictFunc());


//    auto predictFunc = predictor->predictFunc();
//    controller->registPredictFunc([&predictFunc,&solver,&imu](Time::TimeStamp timestamp) {
//        auto result = predictFunc(timestamp);
//        return result;
//    });


// 配置串口和相机
    SerialConfig config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig cameraConfig{
        .cameraSN = param["camera_id"].String(), //前后相机
        .autoWhiteBalance = param["auto_white_balance"].Bool(),//白平衡开启
        .exposureTime = param["exposure_time"].Double(),
        .gain = param["gain"].Double()
    };
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    // 注册串口读取回调函数，该函数在收到新串口数据时被调用
    driver->registReadCallback([control_func = driver->sendSerialFunc() ,controller, &buff_success, buff_controller, &hitBuff, buff_pitch, buff_yaw, shoot_enable,force_shoot,record_enable](const ParsedSerialData& parsedData) {
        
        ControlResult result;  
        if(record_enable)
        {
            Recorder::instance().addSerialData(parsedData);
        }
        //xjj
        if(!hitBuff)
        {
            //如果不打符，调用常规控制器计算控制结果。
            result = controller->control(parsedData);
            if(force_shoot) result.shoot_flag = 1;
            // 如果强制射击，则设置射击标志
            result.shoot_flag = shoot_enable? result.shoot_flag : 0;
            // 根据配置决定是否允许射击
            control_func(result);
            std::cout<<"7777"<<std::endl;
        }
        else //buff
        {
            // 调用大能量机关控制器计算控制结果
            std::cout<<"8888888888888"<<std::endl;
            BuffControlResult buff_result = buff_controller.buff_control(parsedData, buff_pitch, buff_yaw);
            if(force_shoot) buff_result.shoot_flag = 1;
            buff_result.shoot_flag = shoot_enable? buff_result.shoot_flag : 0;
            
            result.shoot_flag = buff_result.shoot_flag;
            result.pitch_setpoint = buff_result.pitch_setpoint;
            result.yaw_setpoint = buff_result.yaw_setpoint;
            result.pitch_actual_want = buff_result.pitch_actual_want;
            result.yaw_actual_want = buff_result.yaw_actual_want;
            std::cout<<"flag:"<<result.shoot_flag<<", pitch_setpoint:"<<result.pitch_setpoint<<", yaw_setpoint:"<<
                result.yaw_setpoint<<", pitch_actual_want:"<<result.pitch_actual_want<<", yaw_actual_want:"<<result.yaw_actual_want<<std::endl;
            result.valid = true;
            control_func(result);
        }
        //xjj
                UdpSend::sendData((float)result.pitch_setpoint);
                UdpSend::sendData((float)result.yaw_setpoint);
                UdpSend::sendData((float)parsedData.pitch_now);
                UdpSend::sendData((float)parsedData.yaw_now);
                UdpSend::sendTail();
    });
    driver->runSerialThread();
    driver->runCameraThread();

    int receive_enemy_color = 0;
    bool isLastDetected = false;
    //Eigen::Vector5d distortionCoefficients = Eigen::Vector5d(-0.0658577209154935, 0.125641157995530, 0, 0, -0.0808951533858169);
    bool autoEnemy = (enemyTrans[param["enemy_color"].String()] == -1);
    if(!autoEnemy)
        detector->setEnemyColor(enemyTrans[param["enemy_color"].String()]);
    Time::TimeStamp lastDetectTime;
    while(1)
    {
        if(!driver->isExistNewCameraData())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            //INFO("Waiting for new camera data...");
            continue;
        }
        clearScreenNoDelete();

        //if multiple frame, get the latest one and discard the others
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        auto deltaTime = (frame->timestamp - lastDetectTime).toSeconds();
        lastDetectTime = frame->timestamp;
        imu = driver->findNearestSerialData(frame->timestamp);
        ImuData imu_data = imu;
        receive_enemy_color = imu.enemy_color;
        INFO("aim_request: {}", imu.aim_request);
        if(autoEnemy)
            detector->setEnemyColor(1 - receive_enemy_color);
        driver->clearSerialData();


        // std::vector<Prediction> predictions;
        // if(predictor->Stable())
        //     predictions = predictor->predict(frame->timestamp);

        //calcROI func will automatically filter out the predictions that are not in the camera view
        // std::vector<XYV> projects;
        // for(const auto &prediction : predictions)
        //     for(const auto &armor : prediction.armors)
        //         if(armor.status != armor.NONEXIST)



        //             projects.push_back(armor.location.getImuCXY(imu_data));
        //auto roi_regions = tracker->calcROI(projects);
        
        // xjj
        if (imu.aim_request == 2 || imu.aim_request == 4) {
            hitBuff = true;
            if (imu.aim_request == 2) 
                buff_mode = 1;
            else if (imu.aim_request == 4) 
                buff_mode = 2;
            else 
                buff_mode = 0;
        }
        else{hitBuff = false; buff_mode = 0;}
        //TODO debug
        // hitBuff = true ;//imu.aim_request == 2;
        // std::cout<<"set ture"<<std::endl;
        // buff_mode = 1; //1小符2大符
        //TODO
        //xjj
        if (!EfficiencyFirst || !hitBuff)//性能优先时不进行装甲板检测   !EfficiencyFirst || !hitBuff
        {
            //TODO
            reload_big_buff = true; //for big buff
            //TODO
            auto detections = detector->detect(frame->image);
            tracker->merge(detections.first);
            tracker->merge(detections.second);
            isLastDetected = tracker->isDetected();

            auto trackResults = tracker->getTrackResult(frame->timestamp, imu);
            for(auto& trackResult : trackResults.first)
            {
                //search car_id in trackResults.second
                auto it = std::find_if(trackResults.second.begin(), trackResults.second.end(), [&trackResult](const auto& armor) {
                    return armor.car_id == trackResult.car_id;
                });
                XYZ xyz_imu;
                double yaw = 0;
                if(it == trackResults.second.end())
                    std::tie(xyz_imu, yaw) = solver->camera2world(trackResult.armor, imu_data, trackResult.car_id == 1);
                else
                {
                    std::tie(xyz_imu, yaw) = solver->camera2worldWithWholeCar(trackResult.armor, imu_data, it->bounding_rect, trackResult.car_id == 1);
                }
                trackResult.location.imu = imu_data;
                trackResult.location.xyz_imu = xyz_imu;
                //XYZ tmp = trackResult.location.xyz_imu;
                //trackResult.location.xyz_imu = tmp; 
                trackResult.yaw = yaw;
                if(draw_debug_image) {
                    CXYD coord = trackResult.location.cxy;
                    cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 12, cv::Scalar(255, 255, 0), -1);
                    location::Location temp;
                    temp.imu = imu_data;
                    XYZ temp_xyz = trackResult.location.xyz_imu;
                    temp_xyz.z = 0;
                    temp.xyz_imu = temp_xyz;
                    CXYD coord_z0 = temp.cxy;
                    cv::line(frame->image, cv::Point(coord.cx, coord.cy), cv::Point(coord_z0.cx, coord_z0.cy),
                            cv::Scalar(255, 255, 0), 2);

                    std::string text = std::to_string(trackResult.yaw);
                    cv::putText(frame->image, text, cv::Point(coord.cx, coord.cy), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(0, 255, 255), 2);
                    std::string text_id = std::to_string(trackResult.armor_id);
                    cv::putText(frame->image, text_id, cv::Point(coord.cx, coord.cy + 10), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                                cv::Scalar(0, 0, 255), 2);
                    XYZ armor_center = trackResult.location.xyz_imu;
                    cv::circle(frame->image, cv::Point2f(500 - armor_center.y * 100.0, 500 - armor_center.x * 100.0), 5,
                            cv::Scalar(0, 0, 255), -1);
                    double k = tan(yaw);
                    double dx = 10 / sqrt(1 + k * k);
                    double dy = k * dx;
                    cv::line(frame->image,
                            cv::Point2f(500 - armor_center.y * 100.0 + dx, 500 - armor_center.x * 100.0 - dy),
                            cv::Point2f(500 - armor_center.y * 100.0 - dx, 500 - armor_center.x * 100.0 + dy),
                            cv::Scalar(0, 0, 255), 2);

                }
            }
            //visualize trackResults on frame
            for(auto& trackResult : trackResults.second)
            {
                auto car_id = trackResult.car_id;
                auto car_type = trackResult.car_type;
                auto bounding_rect = trackResult.bounding_rect;
                if(draw_debug_image) {
                    cv::rectangle(frame->image, bounding_rect, cv::Scalar(255, 0, 0), 5);
                    cv::putText(frame->image, std::to_string(car_id), bounding_rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(255, 0, 0), 5);
                    cv::putText(frame->image, std::to_string(car_type), bounding_rect.tl() + cv::Point2f(0, 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 5);
                }
            }
            INFO("TrackResults size: {}", trackResults.first.size());
            predictor->update(trackResults, frame->timestamp);

        if(draw_debug_image) {
            //DEBUG
            auto predictions = predictor->predict(frame->timestamp + 100ms);
            //visualize predictions on frame
            bool show = false;
            for (auto &prediction: predictions) {
                show = true;
                XYZ center = prediction.center;
                //UdpSend::sendData((float) center.x);
                //UdpSend::sendData((float) center.y);
                //UdpSend::sendData((float) prediction.z1);
                //UdpSend::sendData((float) prediction.z2);
                //UdpSend::sendData((float) prediction.omega);
                //UdpSend::sendData((float) prediction.vx);
                //UdpSend::sendData((float) prediction.vy);
                //UdpSend::sendData((float) prediction.theta);


                INFO("ENTER center: x: {}, y: {}, z: {}", center.x, center.y, center.z);
                //trans to CXYD
                location::Location temp;
                temp.imu = imu_data;
                temp.xyz_imu = center;
                CXYD coord = temp.cxy;
                INFO("ENTER coord: cx: {}, cy: {}", coord.cx, coord.cy);
                cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 8, cv::Scalar(200, 200, 200), -1);
                for (auto &armor: prediction.armors) {
                    auto armor_center = armor.center;
                    temp.xyz_imu = armor_center;
                    CXYD armor_coord = temp.cxy;
                    if (armor.status == armor.UNSEEN)
                        cv::circle(frame->image, cv::Point(armor_coord.cx, armor_coord.cy), 5, cv::Scalar(0, 0, 255),
                                   -1);
                    else
                        cv::circle(frame->image, cv::Point(armor_coord.cx, armor_coord.cy), 10,
                                   cv::Scalar(100, 255, 100), -1);
                    armor_center.z = 0;
                    temp.xyz_imu = armor_center;
                    CXYD armor_coord_z0 = temp.cxy;
                    cv::line(frame->image, cv::Point(armor_coord.cx, armor_coord.cy),
                             cv::Point(armor_coord_z0.cx, armor_coord_z0.cy), cv::Scalar(255, 255, 0), 2);
                    std::string text = std::to_string(armor.yaw - temp.imu.yaw);
                                        cv::putText(frame->image, text, cv::Point(armor_coord.cx, armor_coord.cy), cv::FONT_HERSHEY_SIMPLEX,
                                0.5, cv::Scalar(0, 255, 255), 2);
                    std::string text_id = std::to_string(armor.id);
                    cv::putText(frame->image, text_id, cv::Point(armor_coord.cx, armor_coord.cy + 10),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);


                    cv::circle(frame->image, cv::Point2f(500 - armor_center.y * 100.0, 500 - armor_center.x * 100.0), 5,
                               cv::Scalar(255, 255, 255), -1);
                    double k = tan(armor.yaw);
                    double dx = 10 / sqrt(1 + k * k);
                    double dy = k * dx;
                    cv::line(frame->image,
                             cv::Point2f(500 - armor_center.y * 100.0 + dx, 500 - armor_center.x * 100.0 - dy),
                             cv::Point2f(500 - armor_center.y * 100.0 - dx, 500 - armor_center.x * 100.0 + dy),
                             cv::Scalar(255, 255, 255), 2);
                }
            }
            //draw cross on (500, 500)
            cv::line(frame->image, cv::Point(500 - 10, 500), cv::Point(500 + 10, 500), cv::Scalar(0, 255, 0), 4);
            cv::line(frame->image, cv::Point(500, 500 - 10), cv::Point(500, 500 + 10), cv::Scalar(0, 255, 0), 4);

            int count_armor_debug = 0;
            if (show)
                for (auto &trackResult: trackResults.first) {
                    XYZ armor_xyz = trackResult.location.xyz_imu;
                    //UdpSend::sendData(armor_xyz.x);
                    INFO("debug_armor x: {}, y: {}, z: {}", armor_xyz.x, armor_xyz.y, armor_xyz.z);
                    //UdpSend::sendData(armor_xyz.y);
                    count_armor_debug++;
                    if (count_armor_debug >= 2)
                        break;
                }
            if (show)
                if (count_armor_debug < 2) {
                    //UdpSend::sendData((float) 0);
                    //UdpSend::sendData((float) 0);
                                    }
            }
        }
        //xjj
        else {
            // buff_detector
            int enemy_color = imu.enemy_color;
            buff_success = false;
            std::cout<<"buff現時："<<imu.pitch_now << ", "<<imu.yaw_now << std::endl;
            if (buff_detector.buffDetect(frame->image, enemy_color) == false) {
                std::cout<<"detectfail"<<std::endl;
                buff_success = false;
            }
            else{
                std::cout<<"detectsuccess"<<std::endl;
                auto buffCameraPoints{buff_detector.getCameraPoints()}; //R标+裝甲板5點
                // buff_calculator
                //这里应該要整入time了
                buff_calculator.buff_frame.set(frame->image, std::chrono::steady_clock::now(), imu.pitch_now, imu.yaw_now, imu.roll_now); //pitch, yaw, roll
                std::cout<<"imu.actual_bullet_speed:"<<imu.actual_bullet_speed<<std::endl;
                bool buffResult = buff_calculator.calculate(buff_calculator.buff_frame, buffCameraPoints, buff_mode, imu.actual_bullet_speed > 20.0 ? imu.actual_bullet_speed*100 : 24.0 * 100 , reload_big_buff); 
                if (!buffResult) {
                    std::cout<<"buff_calculator fail"<<std::endl;
                    *buff_pitch = imu.pitch_now + 90.0;
                    *buff_yaw = imu.yaw_now + 90.0;
                    buff_success = false;
                }
                else{
                    std::cout<<"完成了buff_calculator"<<std::endl;
                    // 更新共享变量
                    *buff_pitch = buff_calculator.get_predictPitch();
                    *buff_yaw = buff_calculator.get_predictYaw();
                    buff_success = true;
                }            
                
            }
        }
        //xjj

        //if(udp_enable)
         //   UdpSend::sendTail();
        if(web_debug_enable)
            VideoStreamer::setFrame(frame->image);
        if(record_enable)
        {
            INFO("ADD FRAME");
            // while(camera_data_pack.size() > 0)
            // {
            //     auto camera_data = camera_data_pack.front();
            //     camera_data_pack.pop();
            //     Recorder::instance().addFrame(camera_data);
            // }
            //debug mode
            Recorder::instance().addFrame(frame);
            INFO("FINISH FRAME");

        }
    }
    VideoStreamer::cleanup();
    if(record_enable)
    {
        Recorder::instance().stop();
        INFO("Recording stopped");
    }
    return 0;
}

/*
配置读取: 首先，它从 config.json 文件中读取所有配置参数，并根据这些参数决定是否启用 UDP、网页调试、录像等功能。
模块实例化: 动态创建了所有核心模块的实例：driver (驱动)、solver (解算器)、controller (控制器)、predictor (预测器)、detector (检测器)、tracker (追踪器)。
硬件线程: 启动了两个关键线程：runSerialThread() 负责处理串口通信（接收 IMU 数据，发送控制指令），runCameraThread() 负责获取相机图像。
主循环: while(1) 循环是程序的核心，它持续获取最新的图像数据，并根据当前任务（打装甲板或大能量机关）来调用不同的处理逻辑。
2. driver - 硬件驱动
driver 模块是程序与外部硬件（相机和串口）的桥梁。

相机数据: 它负责从相机中抓取图像，并将图像与对应的时间戳绑定。

串口数据: 它持续接收来自机器人的 IMU 数据（姿态、颜色、请求等），并将这些数据与时间戳同步。

数据同步: findNearestSerialData() 函数是一个关键功能，它根据图像的时间戳找到最接近的串口数据，确保视觉处理和姿态数据是同步的。

回调函数: registReadCallback 注册的回调函数是串口数据驱动的控制逻辑。每当有新的串口数据到达，它就会调用这个回调来计算和发送控制指令。

3. detector - 目标检测
detector 模块负责在图像中识别出装甲板和整车。

模型加载: 可能会加载深度学习模型 (.onnx 或其他格式) 来进行目标识别。

检测功能: detect(frame->image) 函数是其核心功能，它接收一帧图像，并返回检测到的所有装甲板和整车的位置信息。

4. tracker - 目标追踪
tracker 模块负责将每一帧中检测到的目标串联起来，形成连续的运动轨迹。

数据合并: merge() 函数将 detector 识别出的新目标数据与现有的追踪目标进行匹配和合并。

轨迹管理: getTrackResult() 返回当前正在被追踪的目标的最新状态，包括它们的 ID、位置、类型等。这使得程序可以持续追踪特定目标，而不是在每一帧都重新识别。

5. solver - 姿态解算
solver 模块是机器人的“空间几何大脑”，负责将二维图像信息转换为三维空间信息。

坐标系转换: camera2world() 函数是其核心功能。它接收一个装甲板在相机图像中的位置，结合 IMU 姿态信息，计算出该装甲板在世界坐标系中的三维位置。

解算策略: 代码中提到了两种解算方法：一种是只用装甲板信息，另一种是结合整车信息 (camera2worldWithWholeCar)，这使得解算结果更准确。

6. predictor - 运动预测
predictor 模块负责根据目标的历史运动轨迹，预测其在未来的位置。

数据更新: update() 函数持续接收来自 tracker 的最新追踪结果，并用这些数据来更新内部的运动模型。

预测功能: predict() 函数是预测的核心，它接收一个未来时间戳，并返回目标在该时间点的预测位置。这使得控制器可以提前瞄准目标，弥补系统延时。

7. controller - 机器人控制
controller 模块负责根据目标位置计算出最终的控制指令。

控制逻辑: control() 函数是其主要接口。它接收最新的 IMU 数据和预测的目标位置，并计算出机器人云台需要转动的俯仰角 (pitch) 和偏航角 (yaw)，以及是否需要发射子弹 (shoot_flag)。

注册预测函数: registPredictFunc() 函数将 predictor 的预测功能注册到 controller 中，使得控制器可以直接调用预测结果来生成控制指令。

8. buff 模块 - 大能量机关处理
这是代码中一个独立且重要的功能模块，专门用于处理大能量机关。

BuffDetector: 负责在图像中检测大能量机关的中心点和扇叶。

BuffCalculator: 接收 BuffDetector 的结果，进行复杂的运动学和弹道学计算，以解算瞄准大能量机关所需的精确俯仰角和偏航角。

BuffController: 根据 BuffCalculator 的结果生成控制指令，并发送给机器人*/