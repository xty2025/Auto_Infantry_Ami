#include "replayer.hpp"
#include <regex>
#include <sstream>
#include <Log/log.hpp>

using namespace replayer;
using namespace aimlog;

// 工厂函数实现
auto replayer::createReplayer(const std::string& videoPath, const std::string& serialPath, bool onlyVideo, float speed) -> std::unique_ptr<Replayer> {
    return std::make_unique<replayer::Replayer>(videoPath, serialPath, onlyVideo, speed);
}

Replayer::Replayer(const std::string& videoPath, const std::string& serialPath, bool onlyVideo, float speed)
    : videoPath_(videoPath), serialPath_(serialPath), 
      serial_running_(false), camera_running_(false),
      existNewCameraData_(false), playbackSpeed_(speed), 
      paused_(false), finished_(false), onlyVideo_(onlyVideo) { 
    
    // 打开视频文件
    videoCapture_.open(videoPath);
    if (!videoCapture_.isOpened()) {
        ERROR("Failed to open video file: {}", videoPath);
        throw std::runtime_error("Failed to open video file");
    }
    
    if(!onlyVideo_){
        // 打开串口数据文件
        serialFile_.open(serialPath);
        if (!serialFile_.is_open()) {
            ERROR("Failed to open serial data file: {}", serialPath);
            throw std::runtime_error("Failed to open serial data file");
        }
        
        // 解析串口数据文件
        if (!parseSerialFile()) {
            ERROR("Failed to parse serial data file");
            throw std::runtime_error("Failed to parse serial data file");
        }
    } 
    else{
        INFO("Only video mode enabled, serial data file will be ignored");
    }
    
    INFO("Replayer initialized with video: {} and serial data: {}", videoPath, serialPath);
}

Replayer::~Replayer() {
    // 停止所有线程
    serial_running_ = false;
    camera_running_ = false;
    
    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    
    if (videoCapture_.isOpened()) {
        videoCapture_.release();
    }
    
    if (serialFile_.is_open()) {
        serialFile_.close();
    }
}

void Replayer::setSerialConfig(SerialConfig config) {
    // 回放模式下忽略串口配置
    WARN("Serial configuration ignored in replay mode");
}

void Replayer::setCameraConfig(CameraConfig config) {
    // 回放模式下忽略相机配置
    WARN("Camera configuration ignored in replay mode");
}

std::function<void(const ControlResult&)> Replayer::sendSerialFunc() {
    // 返回一个空操作函数，不实际发送控制指令
    return [](const ControlResult& result) {
        INFO("Control command received in replay mode (ignored): pitch={}, yaw={}, valid={}, shoot={}",
            result.pitch_setpoint, result.yaw_setpoint, result.valid, result.shoot_flag);
    };
}

void Replayer::registReadCallback(std::function<void(const ParsedSerialData&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    read_callback_ = std::move(callback);
}

void Replayer::runSerialThread() {
    if (serial_running_) {
        WARN("Serial replay thread already running");
        return;
    }
    
    serial_running_ = true;
    serial_thread_ = std::thread([this]() {
        if (onlyVideo_) {
            INFO("OnlyVideo mode: starting simulation of serial data every 4ms");
            while (serial_running_) {
                if (paused_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                // 模拟产生一个固定值的串口数据
                ParsedSerialData simulatedData;
                simulatedData.pitch_now = 0.0;
                simulatedData.yaw_now = 0.0;
                simulatedData.roll_now = 0.0;
                simulatedData.actual_bullet_speed = 100.0;
                simulatedData.aim_request = false;
                simulatedData.mode_want = 0;
                simulatedData.number_want = 0;
                simulatedData.enemy_color = 0;
                simulatedData.timestamp = Time::TimeStamp::now(); // 或者采用其他固定策略
                
                // 调用回调函数
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (read_callback_) {
                        read_callback_(simulatedData);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }
        } else {
            INFO("Serial replay thread started");
            
            // 重置时间戳基准
            auto startTime = std::chrono::steady_clock::now();
            Time::TimeStamp firstTimestamp;
            
            if (!serial_data_queue_.empty()) {
                firstTimestamp = serial_data_queue_.front().timestamp;
            }
            
            while (serial_running_ && !serial_data_queue_.empty()) {
                if (paused_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                
                auto& serialData = serial_data_queue_.front();
                //search nearest data in the queue
                for(auto& serialData_= serial_data_queue_.front(); 1; serialData_ = serial_data_queue_.front()){
                    
                    // 计算应该等待的时间
                    double elapsedRealSeconds = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - startTime).count();
                    double elapsedSimSeconds = (serialData_.timestamp - firstTimestamp).toSeconds() / playbackSpeed_;
                    if (elapsedRealSeconds < elapsedSimSeconds)
                    {
                        serialData = serialData_;   
                        break;
                    }
                    for(int i=0;i<10;i++)
                        serial_data_queue_.pop();
                }
                
                // 计算应该等待的时间
                double elapsedRealSeconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - startTime).count();
                double elapsedSimSeconds = (serialData.timestamp - firstTimestamp).toSeconds() / playbackSpeed_;
                
                // 如果实际时间小于模拟时间，则等待
                while (elapsedRealSeconds < elapsedSimSeconds) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    elapsedRealSeconds = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - startTime).count();
                    
                    // 检查线程是否应该终止
                    if (!serial_running_ || paused_) break;
                }
                
                // 如果线程被中断则跳出
                if (!serial_running_) break;
                if (paused_) continue;
                
                // 处理当前帧
                {
                    std::lock_guard<std::mutex> lock(serial_data_mutex_);
                    // 存储到数据映射
                    serial_data_map_[serialData.timestamp] = serialData.data;
                }
                
                // 调用回调函数
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (read_callback_) {
                        read_callback_(serialData.data);
                    }
                }
                
                // 移除已处理的数据
                serial_data_queue_.pop();
            }
            
            // 如果队列处理完毕，标记回放完成
            if (serial_data_queue_.empty()) {
                INFO("Serial replay completed");
                
                // 检查相机线程是否也完成，如果是，则设置 finished_ 标志
                if (!camera_running_) {
                    finished_ = true;
                }
            }
            
            serial_running_ = false;
        }
    });
}

void Replayer::runCameraThread() {
    if (camera_running_) {
        WARN("Camera replay thread already running");
        return;
    }
    
    camera_running_ = true;
    camera_thread_ = std::thread([this]() {
        INFO("Camera replay thread started");
        
        // 设定 time base
        auto startTime = std::chrono::steady_clock::now();
        double fps = videoCapture_.get(cv::CAP_PROP_FPS);
        if (fps <= 0) {
            fps = 25.0; // 默认帧率25fps
            WARN("Failed to get video FPS. Using default {} fps", fps);
        }
        double frameInterval = 1.0 / fps;  // 单位：秒
        
        int frameIndex = 0;
        bool isFirstFrame = true;
        Time::TimeStamp firstTimestamp;
        
        while (camera_running_ && videoCapture_.isOpened()) {
            if (paused_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            
            cv::Mat frame;
            if (!videoCapture_.read(frame)) {
                INFO("End of video file reached");
                break;
            }
            
            Time::TimeStamp timestamp;
            if (onlyVideo_) {
                // 仅视频模式下根据帧率生成时间戳
                if (isFirstFrame) {
                    firstTimestamp = Time::TimeStamp::now();
                    timestamp = firstTimestamp;
                    isFirstFrame = false;
                } else {
                    // 计算期望的 timestamp：firstTimestamp + frameIndex * frameInterval
                    timestamp = firstTimestamp + frameInterval * frameIndex;
                }
                INFO("OnlyVideo mode: generated timestamp: {}", timestamp.toString());
            } else {
                // 非 onlyVideo 模式，从串口文件中读取时间戳（原有逻辑）
                std::string timestampStr;
                bool foundValidTimestamp = false;
                while (!foundValidTimestamp && !serialFile_.eof()) {
                    std::getline(serialFile_, timestampStr);
                    if (timestampStr.substr(0, 2) == "F ") {
                        if (timestampStr.find("[skipped]") == std::string::npos) {
                            foundValidTimestamp = true;
                        } else {
                            INFO("Found skipped frame timestamp: {}", timestampStr);
                        }
                    }
                }
                if (!foundValidTimestamp) {
                    WARN("No valid frame timestamp found in file");
                    break;
                }
                std::string cleanTimestampStr = timestampStr.substr(2);
                size_t bracketPos = cleanTimestampStr.find('[');
                if (bracketPos != std::string::npos) {
                    cleanTimestampStr = cleanTimestampStr.substr(0, bracketPos);
                    cleanTimestampStr.erase(cleanTimestampStr.find_last_not_of(" \t") + 1);
                    INFO("Cleaned timestamp: {}", cleanTimestampStr);
                }
                try {
                    timestamp.fromString(cleanTimestampStr);
                } catch (const std::exception& e) {
                    WARN("Failed to parse timestamp: {}, error: {}", cleanTimestampStr, e.what());
                    timestamp = Time::TimeStamp::now();
                    WARN("Using current time as timestamp");
                }
                if (isFirstFrame) {
                    firstTimestamp = timestamp;
                    isFirstFrame = false;
                    INFO("First frame timestamp: {}", firstTimestamp.toString());
                }
            }
            
            // 等待：计算应等待间隔（考虑 playbackSpeed_）
            double elapsedRealSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - startTime).count();
            double elapsedSimSeconds;
            if (onlyVideo_) {
                // 基于生成的帧索引计算应等待的模拟时间
                elapsedSimSeconds = frameIndex * frameInterval / playbackSpeed_;
            } else {
                elapsedSimSeconds = (timestamp - firstTimestamp).toSeconds() / playbackSpeed_;
            }
            
            while (elapsedRealSeconds < elapsedSimSeconds) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                elapsedRealSeconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - startTime).count();
                if (!camera_running_ || paused_) break;
            }
            
            if (!camera_running_) break;
            if (paused_) continue;
            
            // 创建数据包并加入队列
            auto camera_data = std::make_shared<TimeImageData>();
            camera_data->image = frame.clone();
            camera_data->timestamp = timestamp;
            INFO("Camera data timestamp: {}", camera_data->timestamp.toString());
            
            {
                std::lock_guard<std::mutex> lock(camera_data_mutex_);
                camera_data_queue_.push(camera_data);
                existNewCameraData_ = true;
            }
            
            if (onlyVideo_) {
                frameIndex++;  // 更新帧计数
            }
        }
        
        INFO("Camera replay completed");
        
        if (!serial_running_) {
            finished_ = true;
        }
        
        camera_running_ = false;
    });
}

bool Replayer::isExistNewCameraData() {
    return existNewCameraData_;
}

void Replayer::getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) {
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    
    INFO("Getting camera data, queue size: {}", camera_data_queue_.size());
    while (!camera_data_queue_.empty()) {
        camera_data_pack.push(camera_data_queue_.front());
        camera_data_queue_.pop();
    }
    
    existNewCameraData_ = false;
}

ParsedSerialData Replayer::findNearestSerialData(const Time::TimeStamp& timestamp) {
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    if(onlyVideo_) {
        INFO("OnlyVideo mode: returning ZERO serial data");
        ParsedSerialData zeroData;
        zeroData.pitch_now = 10.0;
        zeroData.yaw_now = 170.0;
        zeroData.roll_now = 0.0;
        zeroData.actual_bullet_speed = 20.0;
        zeroData.aim_request = false;
        zeroData.mode_want = 0;
        zeroData.number_want = 0;
        zeroData.enemy_color = 0;
        zeroData.timestamp = Time::TimeStamp();
        return zeroData;
    }
    // 如果没有数据，返回默认值
    if (serial_data_map_.empty()) {
        return ParsedSerialData();
    }
    
    // 找到不小于请求时间戳的第一个元素
    auto it = serial_data_map_.lower_bound(timestamp);
    
    // 如果它是第一个元素，直接返回
    if (it == serial_data_map_.begin()) {
        return it->second;
    }
    
    // 如果找到的是尾后元素，返回最后一个
    if (it == serial_data_map_.end()) {
        --it;
        return it->second;
    }
    
    // 否则，比较前一个元素是否更近
    auto prev = std::prev(it);
    if ((timestamp - prev->first).toSeconds() < (it->first - timestamp).toSeconds()) {
        return prev->second;
    } else {
        return it->second;
    }
}

void Replayer::clearSerialData() {
    // 在回放模式下不清除数据
    WARN("ClearSerialData called in replay mode (ignored)");
}

void Replayer::setPlaybackSpeed(float speed) {
    if (speed <= 0) {
        WARN("Invalid playback speed: {}. Must be positive.", speed);
        return;
    }
    
    playbackSpeed_ = speed;
    INFO("Playback speed set to {}", speed);
}

void Replayer::pause() {
    paused_ = true;
    INFO("Playback paused");
}

void Replayer::resume() {
    paused_ = false;
    INFO("Playback resumed");
}

void Replayer::restart() {
    // 停止当前线程
    serial_running_ = false;
    camera_running_ = false;
    
    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    
    // 清除数据
    {
        std::lock_guard<std::mutex> lock1(camera_data_mutex_);
        std::lock_guard<std::mutex> lock2(serial_data_mutex_);
        
        while (!camera_data_queue_.empty()) {
            camera_data_queue_.pop();
        }
        
        serial_data_map_.clear();
    }
    
    // 重新打开文件
    videoCapture_.release();
    videoCapture_.open(videoPath_);
    
    serialFile_.close();
    serialFile_.open(serialPath_);
    
    // 重新解析串口数据
    parseSerialFile();
    
    // 重置状态
    finished_ = false;
    existNewCameraData_ = false;
    
    INFO("Playback restarted");
}

bool Replayer::isPlaybackFinished() const {
    return finished_;
}

bool Replayer::parseSerialFile() {
    if (!serialFile_.is_open()) {
        ERROR("Serial file not open");
        return false;
    }
    
    // 清空现有数据
    while (!serial_data_queue_.empty()) {
        serial_data_queue_.pop();
    }
    
    // 重置文件指针到开始
    serialFile_.clear();
    serialFile_.seekg(0);
    
    std::string line;
    while (std::getline(serialFile_, line)) {
        // 忽略空行和注释
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // 忽略帧记录行
        if (line.substr(0, 2) == "F ") {
            continue;
        }
        
        // 解析串口数据行
        if (line.substr(0, 2) == "S ") {
            ReplaySerialData serialData;
            serialData.data = parseSerialLine(line.substr(2), serialData.timestamp);
            serial_data_queue_.push(serialData);
        }
    }
    
    // 重置文件指针到开始，以便后续读取帧时间戳
    serialFile_.clear();
    serialFile_.seekg(0);
    
    INFO("Parsed {} serial data entries", serial_data_queue_.size());
    return true;
}

ParsedSerialData Replayer::parseSerialLine(const std::string& line, Time::TimeStamp& timestamp) {
    ParsedSerialData data;
    std::istringstream ss(line);
    
    // 读取时间戳
    std::string timestampStr;
    ss >> timestampStr;
    timestamp.fromString(timestampStr);
    
    // 读取串口数据字段
    ss >> data.pitch_now >> data.yaw_now >> data.roll_now 
       >> data.actual_bullet_speed 
       >> data.aim_request >> data.mode_want 
       >> data.number_want >> data.enemy_color;
    
    // 设置时间戳
    data.timestamp = timestamp;
    
    return data;
}