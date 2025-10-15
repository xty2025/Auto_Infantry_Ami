#include "camera.hpp"
#include "Log/log.hpp"
using namespace aimlog;
using namespace camera;

std::map<CameraConfig::LightSource, GX_AWB_LAMP_HOUSE_ENTRY> light_source_map = {
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_ADAPTIVE, GX_AWB_LAMP_HOUSE_ADAPTIVE},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_FLUORESCENCE, GX_AWB_LAMP_HOUSE_FLUORESCENCE},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_INCANDESCENT, GX_AWB_LAMP_HOUSE_INCANDESCENT},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_U30, GX_AWB_LAMP_HOUSE_U30},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_D50, GX_AWB_LAMP_HOUSE_D50},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_D65, GX_AWB_LAMP_HOUSE_D65},
    {CameraConfig::LightSource::GX_AWB_LAMP_HOUSE_D75, GX_AWB_LAMP_HOUSE_D75}
};

auto camera::createCamera() -> std::unique_ptr<Camera> {
    return std::make_unique<camera::Camera>();
}


void Camera::setCameraConfig(camera::CameraConfig config){
    camera_config_ = config;
    constexpr auto Width = 1280;
    constexpr auto Height = 1024;

    try{
        daheng_camera->initLib();
        daheng_camera->openDevice(config.cameraSN.c_str());
        daheng_camera->setRoiParam(Width,Height,0,0);
        daheng_camera->setExposureGainParam(config.autoExposure,
            config.autoGain,
            config.exposureTime,
            config.autoExposureTimeMin,
            config.autoExposureTimeMax,
            config.gain,
            config.autoGainMin,
            config.autoGainMax,
            config.grayValueMin,
            config.grayValueMax,
            config.triggerSourceLine2);
        daheng_camera->setWhiteBalanceParam(config.autoWhiteBalance,
            config.balanceRatioRed, 
            config.balanceRatioBlue, 
            config.balanceRatioGreen, 
            light_source_map[config.lightSource]);
        daheng_camera->setAAROIParam(Width / 2, Height / 2, 320, 256);
        daheng_camera->acquisitionStart();
    }catch(const std::exception& e) 
    {
        ERROR("Camera configuration failed {}:{}",config.cameraSN,e.what());
    }
}

void Camera::setCameraExposureTime(int exposureTime) {
    if(exposureTime > 0)
    {
        camera_config_.exposureTime = exposureTime;
    }else
    {
        camera_config_.autoExposure = true;
    }
    try{
        daheng_camera->setExposureGainParam(camera_config_.autoExposure,
                                            camera_config_.autoGain,
                                            camera_config_.exposureTime,
                                            camera_config_.autoExposureTimeMin,
                                            camera_config_.autoExposureTimeMax,
                                            camera_config_.gain,
                                            camera_config_.autoGainMin,
                                            camera_config_.autoGainMax,
                                            camera_config_.grayValueMin,
                                            camera_config_.grayValueMax,
                                            camera_config_.triggerSourceLine2);
    }catch(const std::exception& e)
    {
        ERROR("Camera set exposure time failed {}:{}",camera_config_.cameraSN,e.what());
    }
}

void Camera::runCameraThread() {
    running_camera_ = true;
    camera_thread_ = std::thread([this]() {
        while(running_camera_)
        {
            std::shared_ptr<TimeImageData> camera_data_ = std::make_shared<TimeImageData>();
            daheng_camera->ProcGetImage(&camera_data_->image, camera_data_->timestamp);
            {
                std::lock_guard<std::mutex> lock(camera_data_mutex_);
                if (camera_data_pack_.size() >= max_camera_data_queue_size_)
                    camera_data_pack_.pop();
                camera_data_pack_.push(camera_data_);
                this->existNewCameraData = true;
            }
        }
    });
}

void Camera::getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) 
{
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    existNewCameraData = false;
    while(!camera_data_pack_.empty())
    {
        camera_data_pack.push(camera_data_pack_.front());
        camera_data_pack_.pop();
    }
}

bool Camera::isExistNewCameraData() {
    return existNewCameraData;
}