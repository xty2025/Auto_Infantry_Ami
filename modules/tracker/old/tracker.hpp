#pragma once
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "type.hpp"
#include "detector/type.hpp"
#include "solver/type.hpp"

namespace tracker {
using detector::Detections;
using detector::CarDetections;
using solver::ImuData;

class Tracker{
private:
    //可优化：自动计算合适的timeRatio
    const double timeRatio = 20.0; // 认为两次检测之间的时间间隔平均为20ms
    const int maxDeadFrames = 10; // 大于10帧dead则删除
    //const int maxMissFrames_CAR = 10; // 大于10帧miss则删除
    //std::map<int,int> car_id_missFrame;
    std::map<int,int> dead; //if find car_id then set to 0, else add 1
    std::vector<std::pair<ArmorXYV, int>> armors;
    std::vector<std::pair<ArmorXYV, int>> armors_gray;

    std::vector<std::tuple<cv::Rect2f, int, int>> old_car_rects;// rect, car_id, car_type
    std::vector<std::tuple<cv::Rect2f, int, int>> car_rects;
    Time::TimeStamp last_time;
    //bool first_track = true;
    
    TrackResults getArmorTrackResult(const Time::TimeStamp& time, const ImuData& imu);
    CarTrackResults getCarTrackResult(const Time::TimeStamp& time, const ImuData& imu, const TrackResults& armor);

public:
    //To keep stability, donot put ROI's result into this.
    void merge(const Detections& detections, double threshold=20.0f);
    void merge(const CarDetections& detections, double threshold=0.9f);
    
    [[deprecated]] std::vector<cv::Rect2i> calcROI(const std::vector<XYV>& projects, int width=416, int height=416, int camera_width=1280, int camera_height=1024);
    // getTrackResult 已替换为匹配算法（基于 TrackerMatcher），接口保持不变
    TrackResultPairs getTrackResult(const Time::TimeStamp& time, const ImuData& imu);
    
    bool isDetected();
};

std::unique_ptr<Tracker> createTracker();

} // namespace tracker
