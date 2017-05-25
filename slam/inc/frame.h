#pragma once

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "parameter.h"

class Frame
{
public:
	Frame() {}
	Frame(const Frame &p_frame);
	Frame(const int32_t p_index, const cv::Mat &p_color_image, const cv::Mat &p_depth_image, const Parameter &p_parameter);

	void SetPointCloud();
	void ReleaseImage();

	std::vector<cv::Mat> GetDescriptorVector() const;
	cv::Point2f GetPoint2D(const int32_t p_index) const;
	cv::Point3f GetPoint3D(const int32_t p_index) const;
	uint16_t GetDepth(const int32_t p_index) const;

	void SetBowVector(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> &p_bow_vocabulary);
	void SetTransformCameraToLast(const Eigen::Isometry3d &p_transform);
	void SetTransformCameraToWorld(const Eigen::Isometry3d &p_transform);
	void SetTransformWorldToCamera(const Eigen::Isometry3d &p_transform);

private:
	void GetKeyPointAndDescriptor();
	void ComputePoint3D();
	void UndistortKeyPoints();

public:
	int32_t id_;
	int32_t key_point_number_;

	cv::Mat bgr_image_;
	cv::Mat depth_image_;

	std::vector<cv::KeyPoint> key_points_;
	cv::Mat descriptors_;

	DBoW2::BowVector bow_vector_;
	DBoW2::FeatureVector feature_vector_;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_;

	Eigen::Isometry3d transform_camera_to_last_;
	Eigen::Isometry3d transform_camera_to_world_;
	Eigen::Isometry3d transform_world_to_camera_;

private:
	std::vector<cv::KeyPoint> key_points_fixed_;

	std::vector<cv::Point2f> point_2d_;
	std::vector<cv::Point3f> point_3d_;
	std::vector<uint16_t> point_depth_;

	int32_t orb_features_max_;
	float orb_scale_;
	int32_t orb_levels_;
	int32_t orb_threshold_init_;
	int32_t orb_threshold_min_;
	int32_t filter_interval_;
	std::string dataset_dir_;

	float camera_fx_;
	float camera_fy_;
	float camera_cx_;
	float camera_cy_;
	float camera_scale_;
	float depth_max_;

	cv::Mat camera_k_;
	cv::Mat camera_d_;
};

inline cv::Point2f Frame::GetPoint2D(const int32_t p_index) const
{
	return point_2d_[p_index];
}

inline cv::Point3f Frame::GetPoint3D(const int32_t p_index) const
{
	return point_3d_[p_index];
}

inline uint16_t Frame::GetDepth(const int32_t p_index) const
{
	return point_depth_[p_index];
}

inline void Frame::SetTransformCameraToLast(const Eigen::Isometry3d &p_transform)
{
	transform_camera_to_last_ = p_transform;
}

inline void Frame::SetTransformCameraToWorld(const Eigen::Isometry3d &p_transform)
{
	transform_camera_to_world_ = p_transform;
	transform_world_to_camera_ = transform_camera_to_world_.inverse();
}

inline void Frame::SetTransformWorldToCamera(const Eigen::Isometry3d &p_transform)
{
	transform_world_to_camera_ = p_transform;
	transform_camera_to_world_ = transform_world_to_camera_.inverse();
}

inline void Frame::SetBowVector(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> &p_bow_vocabulary)
{
	if (bow_vector_.empty())
	{
		p_bow_vocabulary.transform(GetDescriptorVector(), bow_vector_, feature_vector_, 4);
	}
}
