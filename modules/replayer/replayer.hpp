#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <queue>
#include <map>
#include <chrono>
#include <TimeStamp/TimeStamp.hpp>
#include "driver/type.hpp"
#include "controller/type.hpp"

namespace replayer {

using driver::ParsedSerialData;
using driver::CameraConfig;
using driver::SerialConfig;
using driver::TimeImageData;
using controller::ControlResult;

// 回放数据帧结构
struct ReplayFrame {
    cv::Mat image;
    Time::TimeStamp timestamp;
};

// 回放串口数据帧结构
struct ReplaySerialData {
    ParsedSerialData data;
    Time::TimeStamp timestamp;
};

class Replayer{
public:
    Replayer() = delete; // 禁止默认构造
    Replayer(const std::string& videoPath, const std::string& serialPath, bool onlyVideo = false, float speed = 1.0f);
    ~Replayer();

    // Driver接口实现
    void setSerialConfig(SerialConfig config);
    void setCameraConfig(CameraConfig config);
    std::function<void(const ControlResult&)> sendSerialFunc();
    void registReadCallback(std::function<void(const ParsedSerialData&)> callback);
    void runSerialThread();
    void runCameraThread();
    bool isExistNewCameraData();
    void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack);
    ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp);
    void clearSerialData();

    // Replayer特有方法
    void setPlaybackSpeed(float speed);
    void pause();
    void resume();
    void restart();
    bool isPlaybackFinished() const;

private:
    // 回放相关数据
    std::string videoPath_;
    std::string serialPath_;
    cv::VideoCapture videoCapture_;
    std::ifstream serialFile_;
    bool onlyVideo_;
    
    // 回放控制
    float playbackSpeed_;
    std::atomic<bool> paused_;
    std::atomic<bool> finished_;
    
    // 线程控制
    std::atomic<bool> serial_running_;   // 串口线程运行标志
    std::atomic<bool> camera_running_;   // 相机线程运行标志
    std::mutex camera_data_mutex_;
    std::mutex serial_data_mutex_;
    std::mutex callback_mutex_;
    std::thread camera_thread_;
    std::thread serial_thread_;
    
    // 数据缓存
    std::queue<std::shared_ptr<TimeImageData>> camera_data_queue_;
    std::map<Time::TimeStamp, ParsedSerialData> serial_data_map_;
    std::queue<ReplaySerialData> serial_data_queue_;
    
    // 回调函数
    std::function<void(const ParsedSerialData&)> read_callback_;
    
    // 状态标志
    std::atomic<bool> existNewCameraData_;
    
    // 解析串口数据文件
    bool parseSerialFile();
    
    // 解析一行串口数据
    ParsedSerialData parseSerialLine(const std::string& line, Time::TimeStamp& timestamp);
};

std::unique_ptr<Replayer> createReplayer(const std::string& videoPath, const std::string& serialPath, bool onlyVideo = false, float speed = 1.0f);
} // namespace replayer