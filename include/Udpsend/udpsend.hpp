// filepath: /root/AutoAim/utils/include/Udpsend/udpsend.hpp
#pragma once
#include <boost/asio.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <stdexcept>
#include <Log/log.hpp>  // 使用 ERROR 宏
#include <Param/param.hpp>
/*
static void disable();//禁用UDP：
// 初始化UDP连接
static UdpSend& instance(const std::string & ip = "", int port = 0);
UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());

// 获取实例（用于内部调用）
UdpSend::instance().bufferData(data);
//发数据：
static void sendData(const float& data);
// 发送云台控制数据
UdpSend::sendData((float)result.pitch_setpoint);
UdpSend::sendData((float)result.yaw_setpoint);
UdpSend::sendData((float)parsedData.pitch_now);
UdpSend::sendData((float)parsedData.yaw_now);
static void sendTail();

#private :
UdpSend(const std::string & ip, int port);
//统一的调用方式：
UdpSend::functionName(parameters);
*/





class UdpSend {
public:
    UdpSend(const UdpSend&) = delete;
    UdpSend& operator=(const UdpSend&) = delete;

    static void disable() {
        _disable = true;
    }

    static UdpSend& instance(const std::string & ip = "", int port = 0) {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!s_instance) {
            if(ip.empty() || port == 0) {
                ERROR("UdpSend 未初始化，请提供有效的 ip 和 port");
            }
            else
                s_instance.reset(new UdpSend(ip, port));
        } else if (!ip.empty() && port != 0) {
            s_instance->updateConnection(ip, port);
        }
        return *s_instance;
    }

    static void sendData(const float& data) {
        if(_disable) {WARN("UdpSend Disabled!"); return;}
        instance().bufferData(data);
    }

    static void sendTail() {
        if(_disable) {WARN("UdpSend Disabled!"); return;}
        unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
        instance().bufferData(reinterpret_cast<const float*>(tail), true);
    }

private:
    UdpSend(const std::string & ip, int port)
            : io_context_(), socket_(io_context_)
    {
        updateConnection(ip, port);
    }

    void updateConnection(const std::string & ip, int port) {
        try {
            boost::asio::ip::udp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(boost::asio::ip::udp::v4(), ip, std::to_string(port));
            endpoint_ = *endpoints.begin();
            if (!socket_.is_open()) {
                socket_.open(boost::asio::ip::udp::v4());
            }
        } catch (const std::exception & e) {
            ERROR("UdpSend updateConnection 错误: {}", e.what());
        }
    }

    void bufferData(const float& data, bool forceSend = false) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.push_back(data);

        if (forceSend || buffer_.size() >= buffer_limit_) {
            flushBuffer();
        }
    }

    void bufferData(const float* data, bool forceSend = false) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.insert(buffer_.end(), data, data + 1);

        if (forceSend || buffer_.size() >= buffer_limit_) {
            flushBuffer();
        }
    }

    void flushBuffer() {
        if (buffer_.empty()) return;

        boost::system::error_code ec;
        auto bytes_sent = socket_.send_to(boost::asio::buffer(buffer_.data(), buffer_.size() * sizeof(float)), endpoint_, 0, ec);
        if (ec) {
            ERROR("UdpSend 发送数据错误: {}", ec.message());
        }
        buffer_.clear();
    }

    boost::asio::io_context io_context_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    inline static std::unique_ptr<UdpSend> s_instance = nullptr;
    inline static std::mutex s_mutex;
    inline static bool _disable = false;

    std::vector<float> buffer_;  // 缓冲区
    std::mutex buffer_mutex_;    // 缓冲区互斥锁
    const size_t buffer_limit_ = 1024;  // 缓冲区大小限制
};