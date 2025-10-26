#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
#include "driver/type.hpp"
//调用：
/*
//AutoAim.cpp
Recoder::instance().strat();
Recoder::instance().addSerialData(parsedData);
Recoder::instance().addFrame(frame);
Recoder::instance().stop();
*/


/*addSerialData() 模板方法用于处理串口数据：

- 直接将数据写入文件，避免队列堆积
- 每10条数据执行一次文件刷新（flush）
- 使用特化方法处理特定类型的数据（如ParsedSerialData）
对于 ParsedSerialData 类型，序列化方法会保存以下数据：

- pitch_now, yaw_now, roll_now（当前姿态）
- actual_bullet_speed（实际子弹速度）
- aim_request, mode_want, number_want, enemy_color（目标请求、模式、编号、敌方颜色*/



/*
后台处理线程
Recorder使用独立线程处理视频帧：
add_Frame()添加视频帧。
通过条件变量等待新帧到达
定期检查是否需要退出
处理视频帧并写入视频文件
同时在串口数据文件中记录每帧的时间戳
文件管理
Recorder会创建两个文件：

1.
### 后台处理线程
Recorder使用独立线程处理视频帧：

- 通过条件变量等待新帧到达
- 定期检查是否需要退出
- 处理视频帧并写入视频文件
- 同时在串口数据文件中记录每帧的时间戳
### 文件管理
视频文件（.avi格式）：保存录制的视频帧
2.
串口数据文件（.txt格式）：保存串口数据和视频帧时间戳，格式如下：
# Recording started at [timestamp]（开始标记）
S [timestamp] [data]（串口数据）
F [timestamp]（视频帧时间戳）
# Recording stopped（结束标记
*/

namespace recording {
using driver::ParsedSerialData;
using driver::TimeImageData;
//串口数据和图像数据
class Recorder {
public:
    static Recorder& instance() {
        static Recorder instance;
        return instance;
    }//单例模式只能有一个Recoder的实例。
    
    void start() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (isRunning_) {
            WARN("Recorder already running");
            return;
        }
        /*
    #include<mutex>
    void start(){
        std::lock_guard<std::mutex>lock(mutex_);
        if(isRunning_){
            return ;
        }
    }*/

        // 创建时间戳文件名
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time);
        char buffer[32];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", &tm);
        currentFileName_ = std::string(buffer);
        
        // 确保目录存在
        std::filesystem::create_directories("../record/video");
        std::filesystem::create_directories("../record/serial");
        
        // 设置视频文件路径
        videoPath_ = "../record/video/" + currentFileName_ + ".avi";
        serialPath_ = "../record/serial/" + currentFileName_ + ".txt";
        
        // 重置状态变量
        flushCounter_ = 0;
        
        // 打开串口数据文件
        serialFile_.open(serialPath_, std::ios::app);
        if (serialFile_.is_open()) {
            serialFile_ << "# Recording started at " << currentFileName_ << "\n";
            serialFile_.flush();
        }
        
        // 启动工作线程
        isRunning_ = true;
        workerThread_ = std::thread(&Recorder::processingLoop, this);
        
        INFO("Recording started: {}", currentFileName_);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!isRunning_) {
                return;
            }
            isRunning_ = false;
        }
        
        condVar_.notify_all();
        
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
        
        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            if (serialFile_.is_open()) {
                serialFile_ << "# Recording stopped\n";
                serialFile_.flush();
                serialFile_.close();
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(videoMutex_);
            if (videoWriter_.isOpened()) {
                videoWriter_.release();
            }
        }
        
        INFO("Recording stopped");
    }

    void addFrame(const std::shared_ptr<TimeImageData>& frame) {
        if (!isRunning_ || !frame) return;
        
        // 直接判断队列长度，超过MAX_QUEUE_SIZE就忽略该帧
        {
            std::lock_guard<std::mutex> lock(videoMutex_);
            if (frameQueue_.size() >= MAX_QUEUE_SIZE) {
                // 记录被丢弃的帧信息
                std::lock_guard<std::mutex> serialLock(serialMutex_);
                if (serialFile_.is_open()) {
                    serialFile_ << "F " << frame->timestamp.toString() << " [skipped]\n";
                }
                WARN("Queue full ({}), frame dropped", frameQueue_.size());
                return;
            }
            
            // 队列未满，添加帧
            frameQueue_.push(frame);
        }
        
        // 唤醒工作线程
        condVar_.notify_one();
    }

    // 使用模板函数处理串口数据，采用值拷贝确保安全
    template<typename T>
    void addSerialData(const T& data) {
        if (!isRunning_) return;
        
        // 创建数据副本
        T dataCopy = data;
        Time::TimeStamp timestamp = dataCopy.timestamp;
        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            // 直接写入文件，避免队列堆积
            if (serialFile_.is_open()) {
                serialFile_ << "S " << timestamp.toString() << " ";
                this->serializeData(dataCopy, serialFile_);
                serialFile_ << "\n";
                
                // 每10条数据执行一次flush
                if (++flushCounter_ >= 10) {
                    serialFile_.flush();
                    flushCounter_ = 0;
                }
            }
        }
    }

private:
    Recorder() : 
        isRunning_(false), 
        flushCounter_(0),
        MAX_QUEUE_SIZE(50)  // 最大队列大小限制，可根据需求调整
    {}
    
    ~Recorder() {
        if (isRunning_) stop();
    }
    
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    // 特化给 ParsedSerialData 的序列化方法
    void serializeData(const ParsedSerialData& data, std::ofstream& out) {
        out << data.pitch_now << " "
            << data.yaw_now << " "
            << data.roll_now << " "
            << data.actual_bullet_speed << " "
            << static_cast<int>(data.aim_request) << " "
            << static_cast<int>(data.mode_want) << " "
            << static_cast<int>(data.number_want) << " "
            << static_cast<int>(data.enemy_color);
    }
    
    // 默认序列化方法，可被特化用于其他数据类型
    template<typename T>
    void serializeData(const T& data, std::ofstream& out) {
        out << "UnknownDataType";
    }

    void processingLoop() {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex_);
            
            // 等待条件变量，超时时间为100ms
            condVar_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return !isRunning_ || !frameQueue_.empty();
            });
            
            // 检查是否应该退出
            if (!isRunning_ && frameQueue_.empty()) {
                break;
            }
            
            lock.unlock(); // 解锁以便添加更多帧
            
            processFrames();
        }
        
        // 处理剩余帧
        processFrames();
    }

    void processFrames() {
        std::queue<std::shared_ptr<TimeImageData>> tempQueue;
        
        {
            std::lock_guard<std::mutex> lock(videoMutex_);
            if (frameQueue_.empty()) return;
            std::swap(tempQueue, frameQueue_);
        }
        
        int framesProcessed = 0;
        
        if (!tempQueue.empty()) {
            auto frameData = tempQueue.front();
            tempQueue.pop();
            framesProcessed++;
            
            // 记录帧时间戳
            {
                std::lock_guard<std::mutex> lock(serialMutex_);
                if (serialFile_.is_open()) {
                    serialFile_ << "F " << frameData->timestamp.toString() << "\n";
                }
            }
            
            // 处理视频帧
            {
                // 如果视频写入器尚未初始化，则初始化它
                if (!videoWriter_.isOpened()) {
                    videoWriter_.open(videoPath_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                                     30, frameData->image.size(), true);
                    if (!videoWriter_.isOpened()) {
                        ERROR("Failed to open video file: {}", videoPath_);
                    }
                }
                
                // 写入帧
                if(videoWriter_.isOpened()) {
                    videoWriter_.write(frameData->image);
                }
            }
        }
        
        // 定期刷新串口数据文件
        if (framesProcessed > 0) {
            std::lock_guard<std::mutex> lock(serialMutex_);
            if (serialFile_.is_open()) {
                serialFile_.flush();
            }
        }
    }

    // 成员变量
    std::string currentFileName_;
    std::string videoPath_;
    std::string serialPath_;
    
    cv::VideoWriter videoWriter_;
    std::ofstream serialFile_;
    
    std::thread workerThread_;
    std::mutex mutex_;
    std::mutex videoMutex_;
    std::mutex serialMutex_;
    std::condition_variable condVar_;
    
    std::queue<std::shared_ptr<TimeImageData>> frameQueue_;
    
    std::atomic<bool> isRunning_;
    int flushCounter_ = 0;           // 文件刷新计数器
    
    const size_t MAX_QUEUE_SIZE;     // 队列最大大小限制
};

} // namespace recording