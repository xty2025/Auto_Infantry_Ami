#define NO_LOG
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

namespace fs = std::filesystem;


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
void cameraCalib(std::unique_ptr<Driver> &driver);
void gimbalCalib(std::unique_ptr<Driver> &driver);
void shootCalib(void);
void rollCalib(void);
void pitchyawCalib(void);

int main() {
    bool run = true;
    param::Param param("../config.json");
    param = param[param["car_name"].String()];
    SerialConfig config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig cameraConfig{
            .cameraSN = param["camera_id"].String(),
            .autoWhiteBalance = param["auto_white_balance"].Bool(),
            .exposureTime = param["exposure_time"].Double(),
            .gain = param["gain"].Double()
    };
    auto driver = createDriver();
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    auto detector = createDetector(param["detector"], false);
    detector->setEnemyColor(enemyTrans["r&b"]);
    auto solver = createSolver(param["solver"]);
    auto controller = createController(param["controller"]);
    auto tracker = createTracker();
    auto predictor = createPredictor();
    location::Location::registerSolver(solver);
    controller->registPredictFunc(predictor->predictFunc());
    driver->runSerialThread();
    driver->runCameraThread();
    while(run)
    {
        std::cout<<"============================="<<std::endl;
        std::cout<<"| 1. 相机标定板标定       \t|"<<std::endl;
        std::cout<<"| 2. 云台响应幅度&延迟校准 \t|"<<std::endl;
        std::cout<<"| 3. 发弹延迟&拨盘速度校准 \t|"<<std::endl;
        std::cout<<"| 4. 相机安装偏移-roll校准\t|"<<std::endl;
        std::cout<<"| 5. 相机安装偏移-py校准  \t|"<<std::endl;
        std::cout<<"| 6. exit              \t|"<<std::endl;
        std::cout<<"============================="<<std::endl;
        int choice;
        std::cin>>choice;
        switch(choice)
        {
            case 1:
                driver->setCameraExposureTime(-1);
                cameraCalib(driver);
                driver->setCameraExposureTime(param["exposure_time"].Double());
                break;
            case 2:
                gimbalCalib(driver);
                break;
            case 3:
                shootCalib();
                break;
            case 4:
                rollCalib();
                break;
            case 5:
                pitchyawCalib();
                break;
            case 6:
                run = false;
                break;
            default:
                std::cout<<"输入错误，请重新输入"<<std::endl;
        }
    }


    return 0;
}

void cameraCalib(std::unique_ptr<Driver> &driver)
{
    VideoStreamer::init();
    std::cout << "相机标定板标定，仅保存图片，自行前往matlab进行计算" << std::endl;
    std::cout << "请将云台下电，程序将每隔2s记录图像数据" << std::endl;
    std::cout << "按下回车键开始标定" << std::endl;
    while (std::cin.get() != '\n'); // 等待回车
    std::cout << "开始标定" << std::endl;
    std::cout << "在控制台输入 'q' 然后按回车键退出" << std::endl << std::endl; // 修改退出提示

    int count = 1;
    std::string path = "../calib/"; // 目标路径

    // --- 解决问题1：创建目录 ---
    try {
        if (!fs::exists(path)) {
            if (fs::create_directories(path)) { // create_directories 会创建所有不存在的父目录
                std::cout << "目录 " << path << " 已创建." << std::endl;
            } else {
                std::cerr << "错误: 无法创建目录 " << path << std::endl;
                return; // 创建失败则退出
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "文件系统错误: " << e.what() << std::endl;
        return;
    }
    // --- 问题1解决完毕 ---

    bool quit_flag = false;
    // --- 解决问题2：使用非阻塞的控制台输入 ---
    // 创建一个线程来监听退出命令
    std::thread input_thread([&quit_flag]() {
        char c;
        while (std::cin.get(c)) {
            if (c == 'q') {
                quit_flag = true;
                break;
            }
            // 可以选择忽略其他输入或给出提示
        }
    });
    input_thread.detach(); // 分离线程，让它在后台运行

    Time::TimeStamp current_time = Time::TimeStamp::now();
    cv::Mat newest_image;
    while (!quit_flag) // 使用标志位来控制循环
    {
        // 等待2秒
        //std::this_thread::sleep_for(std::chrono::seconds(2)); // 使用 C++ 标准库延时
        if(!driver->isExistNewCameraData())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        if(!camera_data_pack.empty())
            newest_image = camera_data_pack.back()->image.clone();
        VideoStreamer::setFrame(newest_image.clone());
        double dt = (Time::TimeStamp::now() - current_time).toSeconds();
        if(dt<=1){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue; // 如果距离上次采集时间小于1秒，则继续等待
        }
        current_time = Time::TimeStamp::now(); // 更新当前时间
        if (quit_flag) { // 在获取相机数据前再次检查
            break;
        }


        std::string filename = "calib_" + std::to_string(count) + ".png";
        if (cv::imwrite(path + filename, newest_image.clone())) {
            std::cout << "\033[F\033[K"; // 清除上一行 (可能在所有终端上不完全兼容)
            std::cout << "Saved: " << filename << std::endl;
            count++;
        } else {
            std::cerr << "错误: 无法保存图像 " << path + filename << std::endl;
        }
        cv::Mat pureWhiteImg = cv::Mat::zeros(newest_image.size(), newest_image.type());
        VideoStreamer::setFrame(pureWhiteImg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "退出标定" << std::endl;

    if (count > 1) { // 至少保存了一张图片才压缩 (原先是 >5)
        std::cout << "正在压缩标定数据..." << std::endl;
        // 确保路径正确，特别是在不同操作系统上
        std::string command;
        command = "rm -f ../calib.zip"; // 删除旧的 calib.zip 文件
        std::cout << "执行删除命令: " << command << std::endl;
        int result = system(command.c_str());
        if (result == 0) {
            std::cout << "删除旧的 calib.zip 完成" << std::endl;
        } else {
            std::cerr << "删除失败，返回码: " << result << std::endl;
        }
        // Linux/macOS
        command = "zip -rj ../calib.zip " + fs::absolute(path).string(); // -j 表示不包含目录结构，只压缩文件
        // 如果要包含 calib 目录，使用 -r
        std::cout << "执行压缩命令: " << command << std::endl;
        result = system(command.c_str());
        if (result == 0) {
            std::cout << "压缩完成: calib.zip" << std::endl;
        } else {
            std::cerr << "压缩失败，返回码: " << result << std::endl;
        }
    }
    std::cout << "Auto remove png file." << std::endl;
    std::string command;
    command = "rm -rf ../calib";
    std::cout << "执行删除命令: " << command << std::endl;
    int result = system(command.c_str());
    if (result == 0) {
        std::cout << "删除完成" << std::endl;
    } else {
        std::cerr << "删除失败，返回码: " << result << std::endl;
    }
}
void gimbalCalib(std::unique_ptr<Driver> &driver)
{
    std::cout<<"云台响应幅度&延迟校准（角速度仅能取顺时针分析）"<<std::endl;
    std::cout<<"车辆上电，按下回车键开始标定"<<std::endl;
    while(std::cin.get() != '\n');
    ParsedSerialData serial_data;
    std::queue<std::pair<double, double>> test_queue;
    double theta = 0.0;
    std::pair<double, double> now_test_data;
    double init_yaw = 0.0;
    bool first_yaw_get = false;
    struct RespondData
    {
        double target_dist;
        double target_omega;
        double result_time_bias;
        double result_amplitude_ratio;
        double result_motion_timecost;
    };
    std::queue<RespondData> respond_queue;

    const int recordBeginPeriod = 5;
    const int recordEndPeriod = 15;
    const int motionEndPeriod = 20;
    const double radius = 0.4;
    const double test_dist[] ={2.0,5.0};
    const double test_omega[] = {0.4, 1.0, 3.0};// xxx r/s
    for(int i = 0; i < sizeof (test_dist)/sizeof(double); i++)
    {
        for(int j = 0; j < sizeof (test_omega)/sizeof(double); j++)
        {
            test_queue.push({test_dist[i], test_omega[j]});
        }
    }
    const int totalsize = test_queue.size();

    struct RecordData
    {
        double send_yaw;
        double recv_yaw;
        Time::TimeStamp send_time;
    };
    std::queue<RecordData> record_queue;
    ControlResult control_data;
    while(!test_queue.empty())
    {
        now_test_data = test_queue.front();
        std::cout<<"["<<(totalsize - test_queue.size() + 1)<<"/"<< totalsize
            <<"]现在测试 dist: "<<now_test_data.first<<", omega: "<<now_test_data.second<<std::endl;
        int period_count = 0;
        RespondData respond_data;
        respond_data.target_dist = now_test_data.first;
        respond_data.target_omega = now_test_data.second;
        respond_data.result_time_bias = 0;
        respond_data.result_amplitude_ratio = 0;
        respond_data.result_motion_timecost = 0;
        Time::TimeStamp time_now = Time::TimeStamp::now();
        //bool jump = false;
        while(period_count<motionEndPeriod)
        {
            bool exist_new_data = driver->getNewestSerialData(serial_data);
            if(!exist_new_data){usleep(100);continue;}
            if(first_yaw_get == false)
            {
                init_yaw = serial_data.yaw_now;
                first_yaw_get = true;
            }
            theta += now_test_data.second * 2 * M_PI * (Time::TimeStamp::now() - time_now).toSeconds();
            time_now = Time::TimeStamp::now();
            if(std::abs(theta) > M_PI / 3.0)
            {
                period_count++;
                while(theta > M_PI/3.0) theta -= M_PI/2.0;
                while(theta < -M_PI/3.0) theta += M_PI/2.0;
                // 算法描述：
                // 1. 找到输出量的最小值和最大值处作为波峰
                // 2. 运动时间=输出量最小值与最大值之间的时间差
                // 3. 幅度比=输出量最值差值与输入量最值差值的比值
                // 4. 延时=输出量最小值与输入量最小值之间的时间差 + 两最大值时间差/2
                RecordData record_data;
                double min_output = std::numeric_limits<double>::max();
                double max_output = std::numeric_limits<double>::lowest();
                Time::TimeStamp min_time, max_time;
                double input_min = std::numeric_limits<double>::max();
                double input_max = std::numeric_limits<double>::lowest();
                Time::TimeStamp input_min_time, input_max_time;

                while (!record_queue.empty()) {
                    record_data = record_queue.front();
                    record_queue.pop();

                    // 找到输出量的最小值和最大值
                    if (record_data.recv_yaw < min_output) {
                        min_output = record_data.recv_yaw;
                        min_time = record_data.send_time;
                    }
                    if (record_data.recv_yaw > max_output) {
                        max_output = record_data.recv_yaw;
                        max_time = record_data.send_time;
                    }

                    // 找到输入量的最小值和最大值
                    if (record_data.send_yaw < input_min) {
                        input_min = record_data.send_yaw;
                        input_min_time = record_data.send_time;
                    }
                    if (record_data.send_yaw > input_max) {
                        input_max = record_data.send_yaw;
                        input_max_time = record_data.send_time;
                    }
                }

// 计算运动时间
//                double motion_time = std::abs((max_time - min_time).toSeconds());
//                if(motion_time > (input_max_time - input_min_time).toSeconds() * 0.5)
//                {
//                    motion_time = (input_max_time - input_min_time).toSeconds() - motion_time;
//                }
                double motion_time = (min_time - max_time).toSeconds();
                if(motion_time < 0)
                    motion_time += (input_max_time - input_min_time).toSeconds();
// 计算幅度比
                double amplitude_ratio = (max_output - min_output) / (input_max - input_min);

// 计算延时
                double delay = (min_time - input_min_time).toSeconds() + (max_time - input_max_time).toSeconds() / 2;
                if(min_time < input_min_time)
                {
                    delay += (input_max_time - input_min_time).toSeconds()/2;
                }
                if(max_time < input_max_time)
                {
                    delay += (input_max_time - min_time).toSeconds()/2;
                }
// 输出结果
                std::cout << "运动时间: " << motion_time*1000 << "毫秒" << std::endl;
                std::cout << "幅度比: " << amplitude_ratio << std::endl;
                std::cout << "延时: " << delay*1000 << " 毫秒" << std::endl;
                respond_data.result_motion_timecost += motion_time;
                respond_data.result_amplitude_ratio += amplitude_ratio;
                respond_data.result_time_bias += delay;
            }
            driver->clearSerialData();
            control_data.yaw_setpoint = atan2(radius * sin(theta), now_test_data.first - radius * cos(theta)) * 180 / M_PI + init_yaw;
            control_data.pitch_setpoint = 0;
            control_data.shoot_flag = 0;
            control_data.valid = true;
            driver->sendSerialData(control_data);
            if((period_count>=recordBeginPeriod) && (period_count<recordEndPeriod))
            {
                record_queue.push({control_data.yaw_setpoint, serial_data.yaw_now, Time::TimeStamp::now()});
            }
        }
        respond_data.result_time_bias /= (recordEndPeriod-recordBeginPeriod);
        respond_data.result_amplitude_ratio /= (recordEndPeriod-recordBeginPeriod);
        respond_data.result_motion_timecost /= (recordEndPeriod-recordBeginPeriod);
        respond_queue.push(respond_data);
        test_queue.pop();
    }
    while(respond_queue.size() > 0)
    {
        auto respond_data = respond_queue.front();
        respond_queue.pop();
        std::cout<<"目标距离: "<<respond_data.target_dist<<", 目标角速度: "<<respond_data.target_omega<<std::endl;
        std::cout<<"运动时间: "<<respond_data.result_motion_timecost*1000<<"毫秒"<<std::endl;
        std::cout<<"幅度比: "<<respond_data.result_amplitude_ratio<<std::endl;
        std::cout<<"延时: "<<respond_data.result_time_bias*1000<<" 毫秒"<<std::endl;
    }
    std::cout<<"标定完成"<<std::endl;
}


// 算法：
// 瞄准装甲板发1~2发弹，记录下发弹时间和装甲板掉识别时间（击打闪烁）减去估算飞行时间
// 拨盘速度由弹速更新判断是否发弹。
void shootCalib(void)
{
    std::cout<<"发弹延迟&拨盘速度校准"<<std::endl;
    std::cout<<"车辆上电，按下回车键开始标定"<<std::endl;

}
void rollCalib(void)
{
    std::cout<<"相机安装偏移-roll校准"<<std::endl;
    std::cout<<"标定完成"<<std::endl;
}

void pitchyawCalib(void)
{
    std::cout<<"相机安装偏移-py校准"<<std::endl;
    std::cout<<"标定完成"<<std::endl;
}