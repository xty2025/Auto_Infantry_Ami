#include "detector.hpp"
#include "Log/log.hpp"

//尾置返回类型：
auto detector::createDetector(param::Param json_param, bool allowGray) -> std::unique_ptr<Detector>
{
    return std::make_unique<detector::Detector>(json_param, allowGray);
}
/*
auto detector::createDetector(param::Param json_param,bool allowGray)->std::unique_ptr<Detector>
{
    return std::make_unique<detector::Dtector>(json_patam,allowGray);
}*/
//std::unique_ptr<Detector> detector::createDetector(param::Param json_param, bool allowGray)

namespace detector
{
    //调用.hpp中的接口。检测整车和装甲板pair。
    std::pair<std::vector<Detection>, std::vector<CarDetection>> Detector::detect(const cv::Mat &image, const cv::Rect &roi) {
        if(image.empty()) {
            ERROR("Image is empty");
            return {};
        }
        // if(image.cols != 1280 || image.rows != 720) {
        //     ERROR("Image size is not 1280x720");
        //     INFO("Image size: {}x{}", image.cols, image.rows);
        //     return {};
        // }
        // 使用异步方式启动ArmorOneStage检测
        /*std::async 用来 启动一个异步任务（后台线程执行函数/可调用对象），返回一个 std::future<T>。
std::launch::async 表示 一定在新线程里并行执行。
如果用 std::launch::deferred，则是 惰性执行，等你调用 .get() 时才在当前线程执行。*/
        auto armor_future = std::async(std::launch::async, [&]() {
            bool use_roi = (roi.size() != cv::Size(0,0));
            return (*armor_one_stage_inferer)(use_roi ? image(roi) : image);
        });//返回取决于armor_one_stage_inferer的返回值
        //[&]接受所有外部变量的引用，包括this指针
        // 使用异步方式启动车辆检测
        auto car_future = std::async(std::launch::async, [&]() {
            return yolo_detector->infer(image);
        });
        
        // 等待ArmorOneStage检测完成并处理结果
        BBoxes detection = armor_future.get();
        //BBoxes detection = BBoxes();
        std::vector<Detection> detections;
        detections.reserve(detection.size());//提前分配内存空间
        //如果检测到关键区域就用关键区域，如果检测不到就用原图。
        const int offset_x = (roi.size() != cv::Size(0,0)) ? roi.x : 0;
        const int offset_y = (roi.size() != cv::Size(0,0)) ? roi.y : 0;
        
        for(auto &det : detection) {
            detections.emplace_back();
            //detections.push_back(det);
            auto &result = detections.back();
            //引用解析边框并设置
            // 设置边界框和中心
            result.bounding_rect = cv::Rect2f(
                det.rect.x + offset_x,
                det.rect.y + offset_y,
                det.rect.width,
                det.rect.height
            );
            result.center = cv::Point2f(
                det.center.x + offset_x,
                det.center.y + offset_y
            );
            /*
            result.tag_id=det.tag_id;
            result.color_id=det.color_id;
            result.corners.reserve(4);
            for(const auto &corner:det.corners){
            result.corners.push_back(cv::Point2f(
                corner.x + offset_x,
                corner.y + offset_y
            ));
            }
            */
            // 高效处理角点
            result.corners.reserve(4);
            for(auto &corner : det.corners) {
                result.corners.push_back(cv::Point2f(
                    corner.x + offset_x,
                    corner.y + offset_y
                ));
            }
            
            // 复制其他属性
            result.tag_id = det.tag_id;
            result.score = det.confidence;
            result.isGray = false;
            //result.isGray = det.color_id/2 == 2;
        }
        
        // 等待车辆检测完成并处理结果
        std::vector<YoloDetection> yolo_detections = car_future.get();
        std::vector<CarDetection> car_detections;
        car_detections.reserve(yolo_detections.size());
        //预分配相同的大小的空间
        //detections.reserve(detection.size());
        for(auto &det : yolo_detections) {
            car_detections.emplace_back();
            //car_detections.push_back(det);
            auto &car_detection = car_detections.back();
            
            car_detection.bounding_rect = cv::Rect2f(det.rect);
            car_detection.center = det.center;
            car_detection.tag_id = det.class_id;
            car_detection.score = det.confidence;
            INFO("yolo car detection: {} {} {} {}", det.rect.x, det.rect.y, det.rect.width, det.rect.height);
            INFO("yolo confidence: {}", det.confidence);

            // 在image上绘制检测框
            cv::rectangle(image, det.rect, cv::Scalar(0, 255, 0), 2);
            std::string label = "Class: " + std::to_string(det.class_id) + " " + std::to_string(det.confidence).substr(0, 4);
            int baseline = 0;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        }
        INFO("find {} yolo detections", car_detections.size());
        INFO("find {} armor detections", detections.size());
        //再在这里画出检测的框图。
        return {detections, car_detections};
    }
}