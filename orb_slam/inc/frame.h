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
	Frame(const int32_t p_index, const Parameter &p_parameter);

	void GetImage(const int32_t p_index);
	void GetKeyPointAndDesciptor();
	inline cv::Point2f GetPoint2D(const int32_t p_index) const;
	inline uint16_t GetDepth(const int32_t p_y, const int32_t p_x) const;

public:
	int32_t id_;
	cv::Mat rgb_image_;
	cv::Mat depth_image_;
	cv::Mat desciptors_;
	std::vector<cv::KeyPoint> key_points_;
	Eigen::Isometry3d transform_;

private:
	int32_t orb_features_max_;
	float orb_scale_;
	int32_t orb_levels_;
	int32_t orb_threshold_init_;
	int32_t orb_threshold_min_;
	std::string dataset_dir_;
};

inline cv::Point2f Frame::GetPoint2D(const int32_t p_index) const
{
	return key_points_[p_index].pt;
}

inline uint16_t Frame::GetDepth(const int32_t p_y, const int32_t p_x) const
{
	return depth_image_.ptr<uint16_t>(p_y)[p_x];
}
