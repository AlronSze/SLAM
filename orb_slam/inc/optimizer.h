#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

class Optimizer {
public:
	static void PnPSolver(const std::vector<cv::Point3f>& p_object_points, const std::vector<cv::Point2f>& p_image_points, const cv::Mat p_camera_k, std::vector<int32_t> & p_inliers_index, Eigen::Isometry3d & p_transform);
};
