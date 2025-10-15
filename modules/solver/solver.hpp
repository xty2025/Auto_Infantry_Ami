#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "type.hpp"
#include "driver/type.hpp"
#include "Location/location.hpp"
#include "tracker/type.hpp"
#include "Param/param.hpp"


namespace solver {
	using driver::ParsedSerialData;
	using tracker::ArmorXYV;
class Solver : public BaseSolver
{
private:
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients;// k1,k2,p1,p2,k3
    Eigen::Vector3d cameraRotation;
    Eigen::Matrix3d cameraRotationMatrix;
    double f_x, f_y, c_x, c_y;
public:
    explicit Solver(param::Param& json_param)
    {
        // Load camera intrinsic matrix from parameter file
        auto intrinsicArray = json_param["camera_intrinsic_matrix"].to<std::vector<std::vector<double>>>();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                cameraIntrinsicMatrix(i, j) = intrinsicArray[i][j];
        f_x = cameraIntrinsicMatrix(0, 0);
        f_y = cameraIntrinsicMatrix(1, 1);
        c_x = cameraIntrinsicMatrix(0, 2);
        c_y = cameraIntrinsicMatrix(1, 2);
        auto distortionCoefficientsArray = json_param["camera_distortion_matrix"].to<std::vector<double>>();
        for (int i = 0; i < 5; ++i)
            distorationCoefficients(i) = distortionCoefficientsArray[i];
        auto cameraOffsetArray = json_param["camera_offset"].to<std::vector<double>>();
        for (int i = 0; i < 3; ++i)
            cameraOffset(i) = cameraOffsetArray[i];
        cameraOffset *= 0.001; // convert to meter
        auto cameraRotationArray = json_param["camera_rotation"].to<std::vector<double>>();
        for (int i = 0; i < 3; ++i)
            cameraRotation(i) = cameraRotationArray[i];
        //use cv::Rodrigues to convert rotation vector to rotation matrix
        cv::Mat rotationVector = cv::Mat(3, 1, CV_64F, cameraRotation.data());
        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_64F);
        cv::Rodrigues(rotationVector, rotationMatrix);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                cameraRotationMatrix(i, j) = rotationMatrix.at<double>(i, j);
    }
    
	inline PYD XYZ2PYD(const XYZ& in) const override
	{
		double distance = sqrt(in.x*in.x + in.y*in.y + in.z*in.z);
		double pitch = asin(in.z/distance);
		double yaw = atan2(in.y, in.x);
		return PYD(pitch, yaw, distance);
	}
	inline XYZ PYD2XYZ(const PYD& in) const override
	{
		XYZ out;
		out.x = in.distance * cos(in.pitch) * cos(in.yaw);
		out.y = in.distance * cos(in.pitch) * sin(in.yaw);
		out.z = in.distance * sin(in.pitch);
		return out;
	}
	inline XYZ CXYD2XYZ(const CXYD& in) const override
	{
		//eliminate distortion
        XYZ out;
        out.x = in.k;
        out.y = (c_x - in.cx) * in.k / f_x;
        out.z = (c_y - in.cy) * in.k / f_y;
		return out;
	}
	inline CXYD XYZ2CXYD(const XYZ& in) const override
	{
        double cx = c_x - in.y / in.x * f_x;
        double cy = c_y - in.z / in.x * f_y;
        //distortion
//        double r2 = cx * cx + cy * cy;
//        double r4 = r2 * r2;
//        double r6 = r4 * r2;
//        double r = 1 + distorationCoefficients(0) * r2 + distorationCoefficients(1) * r4 + distorationCoefficients(4) * r6;
//        CXYD out;
//        out.cx = cx * r + 2 * distorationCoefficients(2) * cx * cy + distorationCoefficients(3) * (r2 + 2 * cx * cx);
//        out.cy = cy * r + 2 * distorationCoefficients(3) * cx * cy + distorationCoefficients(2) * (r2 + 2 * cy * cy);
//        out.k = in.x;
        CXYD out;
        out.cx = cx;
        out.cy = cy;
        out.k = in.x;
		return out;
	}
    inline XYZ camera2world(const XYZ& in, const PYD& imuData) const override
    {
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        //double imu_roll = 0;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = cameraRotationMatrix * in_eigen + cameraOffset;
        Eigen::Vector3d world = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                * gimbal;
        return XYZ(world(0), world(1), world(2));
    }
    inline XYZ world2camera(const XYZ& in, const PYD& imuData) const override
    {
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                 * Eigen::AngleAxisd(-imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                 * in_eigen;
        Eigen::Vector3d camera = cameraRotationMatrix.inverse() * (gimbal - cameraOffset);

        return XYZ(camera(0), camera(1), camera(2));
    }
	
	std::pair<XYZ,double> camera2world(const ArmorXYV& trackResult, const ImuData& imuData, bool isLarge);
    std::pair<XYZ,double> camera2worldWithWholeCar(const ArmorXYV& trackResult, const ImuData& imuData, const cv::Rect& bounding_rect, bool isLarge);

	void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d& cameraOffset);
    void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients);
};
std::shared_ptr<Solver> createSolver(param::Param json_param);

} // namespace solver
