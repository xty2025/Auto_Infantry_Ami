#pragma once
#include "Log/log.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <stdio.h>
#include "solver/type.hpp"
#include "tracker/type.hpp"


namespace recordSolver {

using solver::ImuData;
using tracker::ArmorXYV;

inline constexpr auto SmallArmorHalfWidth = 0.0675f;
inline constexpr auto SmallArmorHalfHeight = 0.0275f;
inline constexpr auto SmallArmorWidthRatio = 1.0f;
inline constexpr auto SmallArmorHeightRatio = 1.0f;
//SAH : Small Armor Half
inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
const std::vector SmallArmorPoints =	// 装甲板放在地上
{
    cv::Point3f(-SAHW, SAHH, 0.0f),
    cv::Point3f(-SAHW, -SAHH, 0.0f),
    cv::Point3f(SAHW, -SAHH, 0.0f),
    cv::Point3f(SAHW, SAHH, 0.0f)
};

inline constexpr auto LargeArmorHalfWidth = 0.116f;
inline constexpr auto LargeArmorHalfHeight = 0.0275f;
inline constexpr auto LargeArmorWidthRatio = 0.87f;
inline constexpr auto LargeArmorHeightRatio = 1.0f;
inline constexpr auto LAHW = LargeArmorHalfWidth * LargeArmorWidthRatio;
inline constexpr auto LAHH = LargeArmorHalfHeight * LargeArmorHeightRatio;
inline constexpr auto LargeArmorXRatio = 0.87f;
const std::vector LargeArmorPoints =
{
    cv::Point3f(-LAHW, LAHH, 0.0f),
    cv::Point3f(-LAHW, -LAHH, 0.0f),
    cv::Point3f(LAHW, -LAHH, 0.0f),
    cv::Point3f(LAHW, LAHH, 0.0f)
};
cv::Matx33d cameraIntrinsicMatrix;
cv::Mat distorationCoefficients;
const char* py_filename = "py_data.txt";
const char* dist_filename = "dist_data.txt";

void recordPitchYawData(const std::vector<ArmorXYV>& trackResults, const ImuData& imuData_deg)
{
    static FILE* fp=fopen(py_filename,"w+");
    auto imu_roll = imuData_deg.roll * M_PI / 180;
    auto imu_pitch = imuData_deg.pitch * M_PI / 180;
    auto imu_yaw = imuData_deg.yaw * M_PI / 180;
    for(const auto& armor : trackResults)
    {
        if(armor.size() != 4){
            WARN("Invalid track result");
            continue;
        }
        std::vector<cv::Point2f> imagePoints;
        for (const auto& xyv : armor) 
        {//temporarily dont consider visibility
            imagePoints.emplace_back(xyv.x, xyv.y);
        }
        double center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
        double center_y = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4;
        INFO("centerx:{} centery:{}",center_x,center_y);
        fprintf(fp,"%lf,%lf,%f,%f\n",center_x,center_y,imu_yaw,imu_pitch);
    }
}

//Use this func must set cameraIntrinsicMatrix and distorationCoefficients
//Use this func also need proper parameters for camera2world pitch&yaw calculation
//So perhaps you need to run recordPitchYawData first
void recordDistData(const std::vector<ArmorXYV>& trackResults, const ImuData& imuData_deg, bool isLarge)
{
    static FILE* fp=fopen(dist_filename,"w+");
    if(isLarge)WARN("Large armor maybe not supported yet");
    auto imu_roll = imuData_deg.roll * M_PI / 180;
    auto imu_pitch = imuData_deg.pitch * M_PI / 180;
    auto imu_yaw = imuData_deg.yaw * M_PI / 180;
    Eigen::Matrix3d R = (Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                * Eigen::AngleAxisd(imu_roll, Eigen::Vector3d::UnitX()).toRotationMatrix());
    for(const auto& armor : trackResults)
    {
        if(armor.size() != 4){
            WARN("Invalid track result");
            continue;
        }
        std::vector<cv::Point2f> imagePoints;
        for (const auto& xyv : armor) 
        {//temporarily dont consider visibility
            imagePoints.emplace_back(xyv.x, xyv.y);
        }
        double center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
        double center_y = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4;
        INFO("centerx:{} centery:{}",center_x,center_y);
        double pitch = imu_pitch - ((center_x - 637.5) * (-3.78e-6) + (center_y - 496.5) * (7.394e-4));
        double yaw = imu_yaw - ((center_x - 637.5) * (7.266e-4) + (center_y - 496.5) * (3e-6));
        auto& objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraIntrinsicMatrix, distorationCoefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        cv::Mat Q_c_matrix;
        cv::Rodrigues(rvec, Q_c_matrix);
        Eigen::Matrix3d Q_c;
        cv::cv2eigen(Q_c_matrix, Q_c);
        Eigen::Matrix3d Q_w = R * Q_c;
        //change Q_w to pitch and yaw
        double pitch_w = std::asin(Q_w(2,1));
        double yaw_w = std::atan2(Q_w(1,1),Q_w(0,1));
        //calculate distance
        Eigen::Vector3d P_c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        double dist = P_c.norm();
        INFO("pitch:{} yaw:{}, dist:{}, aim_pitch:{}, aim_yaw:{}",pitch,yaw,dist,pitch_w,yaw_w);
        fprintf(fp,"%lf,%lf,%lf,%lf,%lf\n",pitch,yaw,dist,pitch_w,yaw_w);
    }
}

void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix_) {
    cv::eigen2cv(cameraIntrinsicMatrix_, cameraIntrinsicMatrix);
}
void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients_) {
    cv::eigen2cv(distorationCoefficients_, distorationCoefficients);
}

} // namespace recordSolver
