#pragma once
#include <thread>
#include <queue>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "driver/type.hpp"
#include "controller/type.hpp"
// #include <spdlog/spdlog.h>
// #include <spdlog/sinks/stdout_color_sinks.h>

namespace serial
{
    using driver::SerialConfig;
    using driver::ParsedSerialData;
    using driver::RawSerialData;
    using driver::RawSerialWriteData;
    using driver::TimeImageData;
    using controller::ControlResult;

    class Serial
    {
        public:
            Serial() : io_context_(), serial_port_(io_context_), running_(false) {};
            ~Serial();

        virtual void setSerialConfig(SerialConfig config);

        virtual std::function<void(const ControlResult&)> sendSerialFunc();
            void registReadCallback(std::function<void(const ParsedSerialData&)> callback);
            void runSerialThread();
            ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp);
            void clearSerialData();
        bool getNewestSerialData(ParsedSerialData& data);

        void sendSerialData(const ControlResult& control_result);
        private:
            bool tryOpenPort(const std::string& port_name, int baud_rate);
            bool reconnectSerialPort();
            std::vector<std::string> available_ports_;
            int baud_rate_ = 115200;
            int max_serial_data_queue_size_ = 1000;
            bool enable_CRC_check = true;
            void stopSerialThread();
            //std::shared_ptr<spdlog::logger> logger;
            boost::asio::io_context io_context_;
            boost::asio::serial_port serial_port_;
            std::thread serial_thread_;
            std::thread check_thread_;
            std::atomic<bool> running_;
            std::mutex serial_data_mutex_;
            std::mutex write_mutex_;
            std::queue<ParsedSerialData> serial_data_queue_;

            std::function<void(const ParsedSerialData&)> read_callback_;

            std::atomic<bool> existNewCameraData;

            void startAsyncRead();
            void handleReadData(RawSerialData& raw_data);
            bool validateCRC(RawSerialData &data);
            void addCRC(RawSerialWriteData& data);

            RawSerialWriteData serializeControlResult(const ControlResult& data);
    };
    std::unique_ptr<Serial> createSerial();

}