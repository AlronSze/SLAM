#include "../inc/frame.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "../3rd_part/orb-slam2/ORBextractor.h"

Frame::Frame() : id_(-1), transform_(Eigen::Isometry3d::Identity())
{
}

Frame::Frame(const Frame & p_frame) :
	id_(p_frame.id_), rgb_image_(p_frame.rgb_image_.clone()), depth_image_(p_frame.depth_image_.clone()), descriptors_(p_frame.descriptors_.clone()),
	key_points_(p_frame.key_points_), transform_(p_frame.transform_), point_3d_(p_frame.point_3d_), key_point_number_(p_frame.key_point_number_),
	orb_features_max_(p_frame.orb_features_max_), orb_scale_(p_frame.orb_scale_), orb_levels_(p_frame.orb_levels_),
	orb_threshold_init_(p_frame.orb_threshold_init_), orb_threshold_min_(p_frame.orb_threshold_min_), dataset_dir_(p_frame.dataset_dir_), 
	camera_fx_(p_frame.camera_fx_), camera_fy_(p_frame.camera_fy_), camera_cx_(p_frame.camera_cx_), camera_cy_(p_frame.camera_cy_), 
	camera_scale_(p_frame.camera_scale_), point_rgb_(p_frame.point_rgb_), point_depth_(p_frame.point_depth_), bow_vector(p_frame.bow_vector)
{
}

Frame::Frame(const int32_t p_index, const Parameter & p_parameter) : transform_(Eigen::Isometry3d::Identity())
{
	orb_features_max_ = p_parameter.kORBFeaturesMax_;
	orb_scale_ = p_parameter.kORBScale_;
	orb_levels_ = p_parameter.kORBLevels_;
	orb_threshold_init_ = p_parameter.kORBThresholdInit_;
	orb_threshold_min_ = p_parameter.kORBThresholdMin_;
	dataset_dir_ = p_parameter.kDatasetDir_;

	camera_fx_ = p_parameter.kCameraParameters_.fx_;
	camera_fy_ = p_parameter.kCameraParameters_.fy_;
	camera_cx_ = p_parameter.kCameraParameters_.cx_;
	camera_cy_ = p_parameter.kCameraParameters_.cy_;
	camera_scale_ = p_parameter.kCameraParameters_.scale_;

	GetImage(p_index);
	GetKeyPointAndDescriptor();
	ComputePoint3D();
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

void Frame::GetKeyPointAndDescriptor()
{
	cv::Mat gray;
	cv::cvtColor(rgb_image_, gray, CV_BGR2GRAY);

	ORB_SLAM2::ORBextractor orb(orb_features_max_, orb_scale_, orb_levels_, orb_threshold_init_, orb_threshold_min_);
	orb(gray, cv::Mat(), key_points_, descriptors_);

	key_point_number_ = (int32_t)key_points_.size();
}

void Frame::ComputePoint3D()
{
	for (int32_t i = 0; i < key_point_number_; i++)
	{
		int32_t point_x = (int32_t)key_points_[i].pt.x;
		int32_t point_y = (int32_t)key_points_[i].pt.y;
		uint16_t depth = depth_image_.ptr<uint16_t>(point_y)[point_x];

		if (depth == 0)
		{
			point_3d_.push_back(cv::Point3f(0, 0, 0));
			point_rgb_.push_back(RGB());
			point_depth_.push_back(depth);
		}
		else
		{
			cv::Point3f point_3f;
			point_3f.z = (float)depth / camera_scale_;
			point_3f.x = ((float)point_x - camera_cx_) * point_3f.z / camera_fx_;
			point_3f.y = ((float)point_y - camera_cy_) * point_3f.z / camera_fy_;
			point_3d_.push_back(point_3f);

			struct RGB rgb;
			rgb.r_ = rgb_image_.ptr<uint8_t>(point_y)[point_x * 3 + 2];
			rgb.g_ = rgb_image_.ptr<uint8_t>(point_y)[point_x * 3 + 1];
			rgb.b_ = rgb_image_.ptr<uint8_t>(point_y)[point_x * 3];
			point_rgb_.push_back(rgb);

			point_depth_.push_back(depth);
		}
	}
}

void Frame::ReleaseImage()
{
	rgb_image_.release();
	depth_image_.release();
}

std::vector<cv::Mat> Frame::GetDescriptorVector() const
{
	std::vector<cv::Mat> result;
	for (int32_t i = 0; i < key_point_number_; i++)
	{
		result.push_back(descriptors_.row(i).clone());
	}
	return result;
}
