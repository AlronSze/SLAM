#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include "../3rd_part/dbow2/BowVector.h"

#include "parameter.h"
#include "map_point.h"

class Frame
{
public:
	Frame();
	Frame(const Frame &p_frame);
	Frame(const int32_t p_index, const Parameter & p_parameter);

	void GetImage(const int32_t p_index);
	void GetKeyPointAndDescriptor();
	void ComputePoint3D();
	void InitializeMapPoints();
	void SetPointCloud();
	void ReleaseImage();
	std::vector<cv::Mat> GetDescriptorVector() const;
	inline cv::Point2f GetPoint2D(const int32_t p_index) const;
	inline cv::Point3f GetPoint3D(const int32_t p_index) const;
	inline uint16_t GetDepth(const int32_t p_y, const int32_t p_x) const;
	inline void SetTransform(const Eigen::Isometry3d & p_transform);
	inline Eigen::Isometry3d GetTransform() const;

	static std::vector<cv::DMatch> MatchTwoFrame(const Frame & p_query_frame, const Frame & p_train_frame, const float p_match_ratio);
	static std::vector<cv::DMatch> DoRansacMatch(const Frame & p_query_frame, const Frame & p_train_frame, const std::vector<cv::DMatch> p_matches);

public:
	int32_t id_;
	int32_t key_point_number_;
	cv::Mat rgb_image_;
	cv::Mat depth_image_;

	float camera_fx_;
	float camera_fy_;
	float camera_cx_;
	float camera_cy_;

	DBoW2::BowVector bow_vector;
	cv::Mat descriptors_;
	std::vector<cv::KeyPoint> key_points_;

	std::vector<MapPoint *> map_points_;

	std::vector<cv::Point2f> point_2d_;
	std::vector<cv::Point3f> point_3d_;
	std::vector<uint16_t> point_depth_;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_;

private:
	int32_t orb_features_max_;
	float orb_scale_;
	int32_t orb_levels_;
	int32_t orb_threshold_init_;
	int32_t orb_threshold_min_;
	int32_t filter_interval_;
	std::string dataset_dir_;

	float camera_scale_;
	float depth_max_;

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

inline Eigen::Isometry3d Frame::GetTransform() const
{
	return transform_;
}
