#include "frame.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "ORBextractor.h"

Frame::Frame() : id_(-1), transform_(Eigen::Isometry3d::Identity())
{
}

Frame::Frame(const int32_t p_index, const Parameter &p_parameter)
{
	orb_features_max_ = p_parameter.kORBFeaturesMax_;
	orb_scale_ = p_parameter.kORBScale_;
	orb_levels_ = p_parameter.kORBLevels_;
	orb_threshold_init_ = p_parameter.kORBThresholdInit_;
	orb_threshold_min_ = p_parameter.kORBThresholdMin_;
	dataset_dir_ = p_parameter.kDatasetDir_;

	GetImage(p_index);
	GetKeyPointAndDesciptor();
}

void Frame::GetImage(const int32_t p_index)
{
	id_ = p_index;

	std::stringstream combine;
	std::string file_name;

	combine << dataset_dir_ << "rgb/" << p_index << ".png";
	combine >> file_name;
	rgb_image_ = cv::imread(file_name);

	combine.clear();
	file_name.clear();

	combine << dataset_dir_ << "depth/" << p_index << ".png";
	combine >> file_name;
	depth_image_ = cv::imread(file_name);

	if (rgb_image_.empty() || depth_image_.empty())
	{
		std::cerr << "Failed to load image: " << p_index << ".png" << std::endl;
		exit(0);
	}
}

void Frame::GetKeyPointAndDesciptor()
{
	cv::Mat gray;
	cv::cvtColor(rgb_image_, gray, CV_BGR2GRAY);

	ORB_SLAM2::ORBextractor orb(orb_features_max_, orb_scale_, orb_levels_, orb_threshold_init_, orb_threshold_min_);
	orb(gray, cv::Mat(), key_points_, desciptors_);
}
