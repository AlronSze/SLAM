#pragma once

#include <iterator>
#include <list>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class FeatureNode
{
public:
	FeatureNode();
	void DivideNode(FeatureNode (&p_nodes)[4]);

public:
	cv::Point2i upper_left_;
	cv::Point2i upper_right_;
	cv::Point2i bottom_left_;
	cv::Point2i bottom_right_;
	std::vector<cv::KeyPoint> keypoints_;
	std::list<FeatureNode>::iterator iterator_;
	bool only_one_;
};

class ORBFeature
{
public:
	ORBFeature(const int32_t p_features_max, const float p_scale_factor, const int32_t p_levels, const int32_t p_fast_threshold_init, const int32_t p_fast_threshold_min);
	void operator()(cv::InputArray p_image, std::vector<cv::KeyPoint>& p_keypoints, cv::OutputArray p_descriptors);

private:
	void ComputePyramid(cv::Mat p_image);
	void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint>> & allKeypoints);
	void ComputeOrientation(const cv::Mat & p_image, std::vector<cv::KeyPoint> & p_keypoints);
	void ComputeDescriptors(const cv::Mat & p_image, const std::vector<cv::KeyPoint> & p_keypoints, cv::Mat & p_descriptors);
	std::vector<cv::KeyPoint> DistributeTree(const std::vector<cv::KeyPoint>& p_distributed_keypoints, const int32_t & p_x_min,
		const int32_t & p_x_max, const int32_t & p_y_min, const int32_t & p_y_max, const int32_t & p_features_number);

private:
	int32_t features_max_;
	float scale_factor_;
	int32_t levels_;
	int32_t fast_threshold_init_;
	int32_t fast_threshold_min_;

	std::vector<cv::Point> pattern_;
	std::vector<cv::Mat> image_pyramid_;
	std::vector<int32_t> features_per_level_;
	std::vector<float> scale_factor_vector_;
	std::vector<int32_t> u_max_;

	const int32_t kPatchSize_;
	const int32_t kHalfPatchSize_;
	const int32_t kEdgeThreshold_;
};
