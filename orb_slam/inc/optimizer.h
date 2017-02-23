#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/frame.h"

class Optimizer
{
public:
	static int32_t PnPSolver(const std::vector<cv::Point3f>& p_object_points, const std::vector<cv::Point2f>& p_image_points, const cv::Mat p_camera_k, std::vector<int8_t> & p_inliers_mask, Eigen::Isometry3d & p_transform);
	static void BundleAdjustment(std::vector<Frame> & p_frames);
};
