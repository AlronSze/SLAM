#include "../inc/frame.h"

#include <iostream>
#include <boost/make_shared.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/orb_feature.h"

Frame::Frame() : id_(-1), transform_(Eigen::Isometry3d::Identity())
{
}

Frame::Frame(const Frame & p_frame) :
	id_(p_frame.id_), bgr_image_(p_frame.bgr_image_.clone()), depth_image_(p_frame.depth_image_.clone()), descriptors_(p_frame.descriptors_.clone()),
	key_points_(p_frame.key_points_), transform_(p_frame.transform_), point_3d_(p_frame.point_3d_), key_point_number_(p_frame.key_point_number_),
	orb_features_max_(p_frame.orb_features_max_), orb_scale_(p_frame.orb_scale_), orb_levels_(p_frame.orb_levels_),
	orb_threshold_init_(p_frame.orb_threshold_init_), orb_threshold_min_(p_frame.orb_threshold_min_), dataset_dir_(p_frame.dataset_dir_), 
	camera_fx_(p_frame.camera_fx_), camera_fy_(p_frame.camera_fy_), camera_cx_(p_frame.camera_cx_), camera_cy_(p_frame.camera_cy_), 
	camera_scale_(p_frame.camera_scale_), point_depth_(p_frame.point_depth_), bow_vector(p_frame.bow_vector), depth_max_(p_frame.depth_max_),
	filter_interval_(p_frame.filter_interval_), point_2d_(p_frame.point_2d_), map_points_(p_frame.map_points_), point_cloud_(p_frame.point_cloud_),
	camera_K_(p_frame.camera_K_), camera_D_(p_frame.camera_D_), key_points_fixed_(p_frame.key_points_fixed_)
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

	cv::Mat temp_K = cv::Mat::eye(3, 3, CV_32F);
	temp_K.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	temp_K.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	temp_K.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	temp_K.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;
	temp_K.copyTo(camera_K_);

	cv::Mat temp_D(5, 1, CV_32F);
	temp_D.at<float>(0) = p_parameter.kCameraParameters_.d0_;
	temp_D.at<float>(1) = p_parameter.kCameraParameters_.d1_;
	temp_D.at<float>(2) = p_parameter.kCameraParameters_.d2_;
	temp_D.at<float>(3) = p_parameter.kCameraParameters_.d3_;
	temp_D.at<float>(4) = p_parameter.kCameraParameters_.d4_;
	temp_D.copyTo(camera_D_);

	camera_fx_ = p_parameter.kCameraParameters_.fx_;
	camera_fy_ = p_parameter.kCameraParameters_.fy_;
	camera_cx_ = p_parameter.kCameraParameters_.cx_;
	camera_cy_ = p_parameter.kCameraParameters_.cy_;
	camera_scale_ = p_parameter.kCameraParameters_.scale_;
	depth_max_ = p_parameter.kFilterDepthMax_;
	filter_interval_ = p_parameter.kFilterInterval_;

	GetImage(p_index);
	GetKeyPointAndDescriptor();
	ComputePoint3D();
}

Frame::Frame(const int32_t p_index, const cv::Mat & p_color_image, const cv::Mat & p_depth_image, const Parameter & p_parameter) :
	transform_(Eigen::Isometry3d::Identity()), id_(p_index)
{
	orb_features_max_ = p_parameter.kORBFeaturesMax_;
	orb_scale_ = p_parameter.kORBScale_;
	orb_levels_ = p_parameter.kORBLevels_;
	orb_threshold_init_ = p_parameter.kORBThresholdInit_;
	orb_threshold_min_ = p_parameter.kORBThresholdMin_;

	cv::Mat temp_K = cv::Mat::eye(3, 3, CV_32F);
	temp_K.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	temp_K.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	temp_K.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	temp_K.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;
	temp_K.copyTo(camera_K_);

	cv::Mat temp_D(5, 1, CV_32F);
	temp_D.at<float>(0) = p_parameter.kCameraParameters_.d0_;
	temp_D.at<float>(1) = p_parameter.kCameraParameters_.d1_;
	temp_D.at<float>(2) = p_parameter.kCameraParameters_.d2_;
	temp_D.at<float>(3) = p_parameter.kCameraParameters_.d3_;
	temp_D.at<float>(4) = p_parameter.kCameraParameters_.d4_;
	temp_D.copyTo(camera_D_);

	camera_fx_ = p_parameter.kCameraParameters_.fx_;
	camera_fy_ = p_parameter.kCameraParameters_.fy_;
	camera_cx_ = p_parameter.kCameraParameters_.cx_;
	camera_cy_ = p_parameter.kCameraParameters_.cy_;
	camera_scale_ = p_parameter.kCameraParameters_.scale_;
	depth_max_ = p_parameter.kFilterDepthMax_;
	filter_interval_ = p_parameter.kFilterInterval_;

	bgr_image_ = p_color_image.clone();
	depth_image_ = p_depth_image.clone();
	
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
	bgr_image_ = cv::imread(file_name);

	combine.clear();
	file_name.clear();

	combine << dataset_dir_ << "depth/" << p_index << ".png";
	combine >> file_name;
	depth_image_ = cv::imread(file_name);

	if (bgr_image_.empty() || depth_image_.empty())
	{
		std::cerr << "Failed to load image: " << p_index << ".png" << std::endl;
		exit(0);
	}
}

void Frame::GetKeyPointAndDescriptor()
{
	cv::Mat gray;
	cv::cvtColor(bgr_image_, gray, CV_BGR2GRAY);

	ORBFeature orb(orb_features_max_, orb_scale_, orb_levels_, orb_threshold_init_, orb_threshold_min_);
	orb(gray, key_points_, descriptors_);

	key_point_number_ = (int32_t)key_points_.size();
}

void Frame::ComputePoint3D()
{
	UndistortKeyPoints();

	point_2d_.reserve(key_point_number_);
	point_3d_.reserve(key_point_number_);
	point_depth_.reserve(key_point_number_);

	for (int32_t i = 0; i < key_point_number_; ++i)
	{
		const int32_t point_x = (int32_t)key_points_[i].pt.x;
		const int32_t point_y = (int32_t)key_points_[i].pt.y;
		const int32_t point_x_fixed = (int32_t)key_points_fixed_[i].pt.x;
		const int32_t point_y_fixed = (int32_t)key_points_fixed_[i].pt.y;
		const uint16_t depth = depth_image_.ptr<uint16_t>(point_y)[point_x];

		if ((depth == 0) || (depth > (uint16_t)(depth_max_ * camera_scale_)))
		{
			point_2d_.push_back(cv::Point2f(0, 0));
			point_3d_.push_back(cv::Point3f(0, 0, 0));
			point_depth_.push_back(0);
		}
		else
		{
			point_2d_.push_back(cv::Point2f((float)point_x_fixed, (float)point_y_fixed));

			cv::Point3f point_3f;
			point_3f.z = (float)depth / camera_scale_;
			point_3f.x = ((float)point_x_fixed - camera_cx_) * point_3f.z / camera_fx_;
			point_3f.y = ((float)point_y_fixed - camera_cy_) * point_3f.z / camera_fy_;
			point_3d_.push_back(point_3f);

			point_depth_.push_back(depth);
		}
	}
}

void Frame::InitializeMapPoints()
{
	map_points_.reserve(key_point_number_);

	for (int32_t i = 0; i < key_point_number_; ++i)
	{
		MapPoint * map_point = new MapPoint();

		if (point_depth_[i] == 0)
		{
			map_point->is_bad_ = true;
		}
		else
		{
			map_point->point_2d_ = point_2d_[i];
			map_point->point_3d_ = point_3d_[i];

			const int32_t point_x = (int32_t)point_2d_[i].x;
			const int32_t point_y = (int32_t)point_2d_[i].y;

			map_point->rgb_r_ = bgr_image_.ptr<uint8_t>(point_y)[point_x * 3 + 2];
			map_point->rgb_g_ = bgr_image_.ptr<uint8_t>(point_y)[point_x * 3 + 1];
			map_point->rgb_b_ = bgr_image_.ptr<uint8_t>(point_y)[point_x * 3];

			map_point->InsertObservation(id_, i);
		}

		map_points_.push_back(map_point);
	}
}

void Frame::ReleaseImage()
{
	bgr_image_.release();
	depth_image_.release();
}

std::vector<cv::Mat> Frame::GetDescriptorVector() const
{
	std::vector<cv::Mat> result;
	for (int32_t i = 0; i < key_point_number_; ++i)
	{
		result.push_back(descriptors_.row(i).clone());
	}
	return result;
}

void Frame::SetPointCloud()
{
	point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >();

	const int32_t rows = depth_image_.rows;
	const int32_t cols = depth_image_.cols;

	for (int32_t y = 0; y < rows; y += filter_interval_)
	{
		for (int32_t x = 0; x < cols; x += filter_interval_)
		{
			uint16_t depth = depth_image_.ptr<uint16_t>(y)[x];
			if ((depth == 0) || (depth > (uint16_t)(depth_max_ * camera_scale_)))
			{
				continue;
			}

			pcl::PointXYZRGBA point_xyzrgb;

			cv::Point3f point_3f;
			point_3f.z = (float)depth / camera_scale_;
			point_3f.x = ((float)x - camera_cx_) * point_3f.z / camera_fx_;
			point_3f.y = ((float)y - camera_cy_) * point_3f.z / camera_fy_;

			point_xyzrgb.b = bgr_image_.ptr<uint8_t>(y)[x * 3];
			point_xyzrgb.g = bgr_image_.ptr<uint8_t>(y)[x * 3 + 1];
			point_xyzrgb.r = bgr_image_.ptr<uint8_t>(y)[x * 3 + 2];

			point_xyzrgb.x = point_3f.x;
			point_xyzrgb.y = point_3f.y;
			point_xyzrgb.z = point_3f.z;

			point_cloud_->points.push_back(point_xyzrgb);
		}
	}
}

void Frame::UndistortKeyPoints()
{
	if (camera_D_.at<float>(0) == 0.0)
	{
		key_points_fixed_ = key_points_;
		return;
	}

	cv::Mat temp(key_point_number_, 2, CV_32F);
	key_points_fixed_.resize(key_point_number_);

	for (int32_t i = 0; i < key_point_number_; ++i)
	{
		temp.at<float>(i, 0) = key_points_[i].pt.x;
		temp.at<float>(i, 1) = key_points_[i].pt.y;
	}

	temp = temp.reshape(2);
	cv::undistortPoints(temp, temp, camera_K_, camera_D_, cv::Mat(), camera_K_);
	temp = temp.reshape(1);

	for (int32_t i = 0; i < key_point_number_; ++i)
	{
		cv::KeyPoint key_point = key_points_[i];
		key_point.pt.x = temp.at<float>(i, 0);
		key_point.pt.y = temp.at<float>(i, 1);
		key_points_fixed_[i] = key_point;
	}
}
