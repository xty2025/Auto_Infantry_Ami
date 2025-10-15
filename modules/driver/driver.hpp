#include "type.hpp"
#include "driver.camera/camera.hpp"
#include "driver.serial/serial.hpp"
#include "controller/type.hpp"

namespace driver
{
    using controller::ControlResult;
    using namespace camera;
    using namespace serial;
    // class Serial
    // {
    // public:
    //     virtual void setSerialConfig(SerialConfig config) = 0;
    //     virtual std::function<void(const ControlResult&)> sendSerialFunc() = 0;
    //     virtual void registReadCallback(std::function<void(const ParsedSerialData&)> callback) = 0;
    //     virtual void runSerialThread() = 0;
    //     virtual ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) = 0;
    //     virtual void clearSerialData() = 0;
    // };
    // std::unique_ptr<Serial> createSerial();

    // class Camera
    // {
    // public:
    //     virtual void setCameraConfig(CameraConfig config) = 0;
    //     virtual void runCameraThread() = 0;
    //     virtual bool isExistNewCameraData() = 0;
    //     virtual void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) = 0;
    // };
    // std::unique_ptr<Camera> createCamera();



    class Driver : public Serial, public Camera
    {
    private:
        std::unique_ptr<Serial> serial;
        std::unique_ptr<Camera> camera;
    public:
        Driver(std::unique_ptr<Serial> serial, std::unique_ptr<Camera> camera): serial(std::move(serial)), camera(std::move(camera)) {};

        //ATTENTION:This function shouldn't be used in AutoAim
        //Just for DEBUG
        [[deprecated("This function is for debug purposes only and should not be used in production.")]]
        bool getNewestSerialData(ParsedSerialData& serial_data);
        [[deprecated("This function is for debug purposes only and should not be used in production.")]]
        void sendSerialData(const ControlResult& control_result);


        void setSerialConfig(SerialConfig config) override;
        void setCameraConfig(CameraConfig config) override;
        void setCameraExposureTime(int exposure_time);
        std::function<void(const ControlResult&)> sendSerialFunc() override;
        void registReadCallback(std::function<void(const ParsedSerialData&)> callback);
        void runSerialThread();
        void runCameraThread();
        bool isExistNewCameraData();
        void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack);
        ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp);
        void clearSerialData();
    };
    std::unique_ptr<Driver> createDriver();
    
} // namespace driver