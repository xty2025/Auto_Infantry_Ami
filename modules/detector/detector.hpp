#pragma once
#include "ArmorOneStage.hpp"
#include "YoloDetector.hpp"
#include "type.hpp"
#include "Param/param.hpp"
#include <future>


namespace detector
{
    class Detector
    {
    private:
        std::string armor_model_path;
        std::string car_model_path;
        bool useOldModel = false;
        std::unique_ptr<ArmorOneStage> armor_one_stage_inferer;
        std::unique_ptr<YoloDetector> yolo_detector;
    public:
        explicit Detector(param::Param json_param, bool allowGray = true)
        {
            armor_model_path = json_param["model_path"].String();
            car_model_path = json_param["car_model_path"].String();
            useOldModel = json_param["use_old_model"].Bool();
            //声明智能指针
            armor_one_stage_inferer = std::make_unique<ArmorOneStage>(armor_model_path, useOldModel, allowGray);
            //单个的ArmorOneStage的检测，重载后的（）运算符，返回的是一个BBoxes。
            yolo_detector = std::make_unique<YoloDetector>(car_model_path);
        }
        ~Detector() {}

        std::pair<std::vector<Detection>, std::vector<CarDetection>> detect(const cv::Mat &image, const cv::Rect &roi = cv::Rect());
        void setEnemyColor(int flag)//0: red, 1: blue
        {
            armor_one_stage_inferer->setColorFlag(flag);
        }
    };
    std::unique_ptr<Detector> createDetector(param::Param json_param, bool allowGray = true);
} // namespace detector