#include "DaHengCamera.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include "driver/type.hpp"

namespace camera
{
    using driver::CameraConfig;
    using driver::TimeImageData;
    class Camera
    {
    public:
        virtual void setCameraConfig(CameraConfig config);
        void runCameraThread();
        bool isExistNewCameraData();
        void setCameraExposureTime(int exposure_time);
        void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack);
    private:
        CameraConfig camera_config_;
        int max_camera_data_queue_size_ = 100;
        std::atomic<bool> running_camera_;
        std::mutex camera_data_mutex_;
        std::thread camera_thread_;
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack_;
        std::unique_ptr<DaHengCamera> daheng_camera = std::make_unique<DaHengCamera>();
        std::atomic<bool> existNewCameraData;
    };
    std::unique_ptr<Camera> createCamera();
}