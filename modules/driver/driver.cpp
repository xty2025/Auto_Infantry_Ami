#include "driver.hpp"
using namespace driver;

//声明一个driver->Driver,后面用Driver的createDriver()函数创建一个Driver对象
auto driver::createDriver()->std::unique_ptr<Driver>{
    return std::make_unique<driver::Driver>(createSerial(), createCamera());
}
/*std::unique_ptr<Driver> auto driver::createDriver(){
    return std::make_unique<driver::Driver>(createSerial(), createCamera());
}*/

/*
//格式化函数：
Type Driver::drive_Func()->std::unique_ptr<Driver>(class &h){
    return this->(if serial)?serial:camera->drive_Func(h);
}    
*/


bool Driver::getNewestSerialData(ParsedSerialData& serial_data)
{
    return this->serial->getNewestSerialData(serial_data);
}

void Driver::sendSerialData(const ControlResult& control_result)
{
    this->serial->sendSerialData(control_result);
}

void Driver::setSerialConfig(SerialConfig config)
{
    this->serial->setSerialConfig(config);
}

void Driver::setCameraConfig(CameraConfig config)
{
    this->camera->setCameraConfig(config);
}

void Driver::setCameraExposureTime(int exposure_time)
{
    this->camera->setCameraExposureTime(exposure_time);
}

std::function<void(const ControlResult&)> Driver::sendSerialFunc()
{
    return this->serial->sendSerialFunc();
}

void Driver::registReadCallback(std::function<void(const ParsedSerialData&)> callback)
{
    this->serial->registReadCallback(callback);
}

void Driver::runSerialThread()
{
    this->serial->runSerialThread();
}

void Driver::runCameraThread()
{
    this->camera->runCameraThread();
}

bool Driver::isExistNewCameraData()
{
    return this->camera->isExistNewCameraData();
}

void Driver::getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack)
{
    return this->camera->getCameraData(camera_data_pack);
}

ParsedSerialData Driver::findNearestSerialData(const Time::TimeStamp& timestamp)
{
    return this->serial->findNearestSerialData(timestamp);
}

void Driver::clearSerialData()
{
    this->serial->clearSerialData();
}

RawSerialWriteData::RawSerialWriteData(const ControlResult& x)
: start_flag('!'), detect_number(static_cast<int>(x.valid)), shoot_flag(x.shoot_flag), 
            pitch_setpoint(x.pitch_setpoint), yaw_setpoint(x.yaw_setpoint) {}
