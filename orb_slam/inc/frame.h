#pragma once

#include "parameter.h"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Frame {
public:
	Frame();
	Frame(const Frame &p_frame);
	Frame(const int32_t p_index, const Parameter & p_parameter);

	void GetImage(const int32_t p_index);
	void GetKeyPointAndDesciptor();
	void ComputePoint3D();
	inline cv::Point2f GetPoint2D(const int32_t p_index) const;
	inline cv::Point3f GetPoint3D(const int32_t p_index) const;
	inline uint16_t GetDepth(const int32_t p_y, const int32_t p_x) const;
	inline void SetTransform(const Eigen::Isometry3d & p_transform);
	inline Eigen::Isometry3d GetTransform();

public:
	int32_t id_;
	cv::Mat rgb_image_;
	cv::Mat depth_image_;
	cv::Mat desciptors_;
	std::vector<cv::KeyPoint> key_points_;
	std::vector<cv::Point3f> point_3d_;
	int32_t key_point_number_;

private:
	int32_t orb_features_max_;
	float orb_scale_;
	int32_t orb_levels_;
	int32_t orb_threshold_init_;
	int32_t orb_threshold_min_;
	std::string dataset_dir_;

	float camera_fx_;
	float camera_fy_;
	float camera_cx_;
	float camera_cy_;
	float camera_scale_;

	Eigen::Isometry3d transform_;
};

inline cv::Point2f Frame::GetPoint2D(const int32_t p_index) const
{
	return key_points_[p_index].pt;
}

inline cv::Point3f Frame::GetPoint3D(const int32_t p_index) const
{
	return point_3d_[p_index];
}

inline uint16_t Frame::GetDepth(const int32_t p_y, const int32_t p_x) const
{
	return depth_image_.ptr<uint16_t>(p_y)[p_x];
}

inline void Frame::SetTransform(const Eigen::Isometry3d & p_transform)
{
	transform_ = p_transform;
}

inline Eigen::Isometry3d Frame::GetTransform()
{
	return transform_;
}
