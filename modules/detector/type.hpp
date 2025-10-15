#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

namespace detector
{
    struct Detection
    {
        cv::Rect2f bounding_rect;
        cv::Point2f center;
        int tag_id;
        float score;
        std::vector<cv::Point2f> corners;
        bool isGray = false;
    };
    typedef std::vector<Detection> Detections;

    struct CarDetection
    {
        cv::Rect2f bounding_rect;
        cv::Point2f center;
        int tag_id;// will be used for recognize Car's Model
        //but now it's not used
        float score;
    };
    typedef std::vector<CarDetection> CarDetections;

    class BBox
    {
    public:
        BBox() : center(0, 0), rect(0, 0, 0, 0), area(0.0f), confidence(0.0f), color_id(0), tag_id(0)
        {
            for(int i = 0; i < 4; i++)
            {
                corners[i] = cv::Point2f(0, 0); // 初始化 corners 数组
            }
            points.clear(); // 清空 points 向量
        }

        cv::Point2f corners[4];
        std::vector<cv::Point2f> points;
        cv::Point2f center;
        cv::Rect rect;
        float area;
        float confidence;
        int color_id;
        int tag_id;
    };
    typedef std::vector<BBox> BBoxes;
};