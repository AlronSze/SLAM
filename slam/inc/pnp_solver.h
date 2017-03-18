#pragma once

#include <vector>

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include "../inc/frame.h"

class PnPSolver
{
public:
	PnPSolver(const int32_t p_inliers_threshold, const float p_match_ratio, const cv::Mat &p_camera_k);

	bool SolvePnP(const Frame &p_query_frame, const Frame &p_train_frame, const int32_t p_pnp_method = 0);
	void SetTransformEstimate(Eigen::Isometry3d &p_transform);
	void ResetInliersThreshold(const int32_t p_inliers_threshold);
	void ResetMatchRatio(const float p_match_ratio);
	Eigen::Isometry3d GetTransform();

private:
	int32_t SolvePnPG2O(const std::vector<cv::Point3f> &p_object_points, const std::vector<cv::Point2f> &p_image_points);

public:
	enum PnPMethod
	{
		USING_G2O = 0,
		USING_EPNP
	};

private:
	int32_t inliers_threshold_;
	float match_ratio_;
	cv::Mat camera_k_;
	Eigen::Isometry3d transform_;
};

inline void PnPSolver::SetTransformEstimate(Eigen::Isometry3d &p_transform)
{
	transform_ = p_transform;
}

inline void PnPSolver::ResetInliersThreshold(const int32_t p_inliers_threshold)
{
	inliers_threshold_ = p_inliers_threshold;
}

inline void PnPSolver::ResetMatchRatio(const float p_match_ratio)
{
	match_ratio_ = p_match_ratio;
}

inline Eigen::Isometry3d PnPSolver::GetTransform()
{
	return transform_;
}
