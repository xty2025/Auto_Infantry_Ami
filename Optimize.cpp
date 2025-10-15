#include <iostream>
#include <modules/modules.hpp>
#include <Param/param.hpp>
#include <Log/log.hpp>
#include <modules/ControlOptimizer/ControlOptimizer.hpp>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>
#include <atomic>

using namespace modules;
using namespace aimlog;

// 全局共享变量，存储最新的云台角度和控制目标
std::mutex angle_mutex, control_mutex;
double current_pitch = 0.0;
double current_yaw = 0.0;
double target_pitch = 0.0;
double target_yaw = 0.0;
std::atomic<bool> running{true};

// 持续发送控制命令的线程函数
void controlThread(std::function<void(ControlResult)> controlFunc) {
    while(running) {
        double pitch, yaw;
        {
            std::lock_guard<std::mutex> lock(control_mutex);
            pitch = target_pitch;
            yaw = target_yaw;
        }

        ControlResult result;
        result.pitch_setpoint = pitch;
        result.yaw_setpoint = yaw;
        result.shoot_flag = 0;  // 不发射

        controlFunc(result);

        // 每10ms发送一次控制命令，保持云台响应
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 获取当前时间字符串，用于文件名
std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S");
    return ss.str();
}

// 将测试结果保存到文件
void saveResultsToFile(const ControlOptimizer::TestResult& results, const std::string& filename) {
    // 保存逻辑不变，省略...
    INFO("测试结果已保存到 {}", filename);
}

int main(int argc, char* argv[]) {
    // 加载配置
    param::Param param("../config.json");
    param = param[param["car_name"].String()];

    INFO("初始化控制优化器测试...");
    auto driver = createDriver();

    // 配置驱动器
    SerialConfig config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig cameraConfig{
            .cameraSN = param["camera_id"].String(),
            .autoWhiteBalance = param["auto_white_balance"].Bool(),
            .exposureTime = param["exposure_time"].Double(),
            .gain = param["gain"].Double()
    };
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);

    // 注册回调函数，用于接收串口数据并更新全局变量
    driver->registReadCallback([](const ParsedSerialData& parsedData) {
        std::lock_guard<std::mutex> lock(angle_mutex);
        current_pitch = parsedData.pitch_now;
        current_yaw = parsedData.yaw_now;
    });

    // 运行串行线程
    driver->runSerialThread();

    // 记录初始位置
    double initialPitch, initialYaw;
    {
        std::lock_guard<std::mutex> lock(angle_mutex);
        initialPitch = current_pitch;
        initialYaw = current_yaw;
    }
    INFO("记录初始位置: Pitch={}, Yaw={}", initialPitch, initialYaw);

    // 初始化目标位置为初始位置
    {
        std::lock_guard<std::mutex> lock(control_mutex);
        target_pitch = initialPitch;
        target_yaw = initialYaw;
    }

    // 获取发送控制命令的函数
    auto controlFunc = driver->sendSerialFunc();

    // 启动持续发送控制命令的线程
    std::thread control_thread(controlThread, controlFunc);
    INFO("控制线程已启动");

    // 创建发送控制命令的函数包装 - 使用偏移量而不是绝对位置
    auto sendControlFunction = [initialPitch, initialYaw](double pitch_offset, double yaw_offset) {
        std::lock_guard<std::mutex> lock(control_mutex);
        target_pitch = initialPitch + pitch_offset;  // 相对于初始位置的偏移
        target_yaw = initialYaw + yaw_offset;        // 相对于初始位置的偏移
    };

    // 创建获取位置的函数 - 返回值需要减去初始位置才是真正的偏移量
    auto getPositionFunction = [initialPitch, initialYaw]() -> std::pair<double, double> {
        std::lock_guard<std::mutex> lock(angle_mutex);
        return {current_pitch - initialPitch, current_yaw - initialYaw};  // 返回相对偏移量
    };

    // 创建ControlOptimizer实例
    INFO("创建控制优化器实例...");
    ControlOptimizer optimizer(sendControlFunction, getPositionFunction);

    // 可选：自定义测试参数
    if (argc > 1 && std::string(argv[1]) == "--custom") {
        optimizer.setTestAmplitudes({1.0, 2.5, 5.0, 8.0});
        optimizer.setSineFrequencies({0.25, 0.5, 1.0, 2.0});
        INFO("使用自定义测试参数");
    }

    // 执行测试
    INFO("开始控制系统测试...");

    // 判断是否要单独测试某个功能
    bool run_all = true;
    if (argc > 1) {
        std::string test_type = argv[1];
        if (test_type == "--deadzone") {
            optimizer.testDeadZone();
            run_all = false;
        } else if (test_type == "--point") {
            optimizer.testPointResponse();
            run_all = false;
        } else if (test_type == "--sine") {
            optimizer.testSineResponse();
            run_all = false;
        } else if (test_type == "--sawtooth") {
            optimizer.testSawtoothResponse();
            run_all = false;
        }
    }

    if (run_all) {
        optimizer.testAll();
    }

    // 获取并保存结果
    const auto& results = optimizer.getResults();
    std::string filename = "control_test_" + getCurrentTimeString() + ".md";
    saveResultsToFile(results, filename);

    // 停止控制线程
    running = false;
    control_thread.join();

    INFO("测试完成。");
    return 0;
}