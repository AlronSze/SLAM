#include "tracking.h"

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/types/sba/types_six_dof_expmap.h>
//#include <g2o/types/slam3d/edge_se3.h>

Tracking::Tracking(const Parameter & p_parameter) : tracking_state_(INITIALIZE), lost_count_(0), cur_inliers_(0), ref_inliers_(0)
{
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

	camera_scale_ = p_parameter.kCameraParameters_.scale_;
	match_ratio_ = p_parameter.kMatchRatio_;
}

void Tracking::GetFrame(Frame * p_frame)
{
	cur_frame_ = p_frame;
	Track();
}

void Tracking::Track()
{
	bool is_tracked = true;

	if (tracking_state_ == INITIALIZE)
	{
		is_tracked = Initialization();
	}
	else if (tracking_state_ == OK)
	{
		is_tracked = TrackWithMotion();
		if (!is_tracked)
		{
			is_tracked = TrackWithRefFrame();
		}
	}
	else
	{
		is_tracked = Relocalization();
	}

	if (is_tracked)
	{
		if (ref_inliers_ == 0)
		{
			ref_inliers_ = cur_inliers_;
		}

		if (((cur_frame_->id_ >= (ref_frames_->id_ + 20)) || ((float)cur_inliers_ < (float)ref_inliers_ * 0.25f)) && ((((float)cur_inliers_ < (float)ref_inliers_ * 0.75f)) && (cur_inliers_ > 15)))
		{
			std::cout << "Insert New Key Frame!" << std::endl;
			key_frames_.push_back(cur_frame_);
			ref_frames_ = cur_frame_;
		}
	}

	last_frame_ = cur_frame_;
}

bool Tracking::Initialization()
{
	if (cur_frame_->key_point_number_ >= 500)
	{
		std::cout << "Tracking Initialized!" << std::endl;
		key_frames_.push_back(cur_frame_);
		ref_frames_ = cur_frame_;
		tracking_state_ = OK;
		return true;
	}
	else
	{
		std::cout << "Tracking Initialization Failed!" << std::endl;
		return false;
	}
}

bool Tracking::TrackWithMotion()
{
	std::vector<cv::DMatch> matches = MatchTwoFrame(cur_frame_, last_frame_);
	int32_t matches_size = matches.size();
	if (matches_size < 20)
	{
		std::cout << "Tracking With Motion Failed!" << std::endl;
		return false;
	}

	cur_inliers_ = OptimizePose(cur_frame_, last_frame_, matches);

	if (cur_inliers_ <= 10)
	{
		std::cout << "Tracking With Motion Failed!" << std::endl;
		return false;
	}

	std::cout << "Tracking With Motion Succeeded!" << std::endl;
	return true;
}

bool Tracking::TrackWithRefFrame()
{
	std::vector<cv::DMatch> matches = MatchTwoFrame(cur_frame_, ref_frames_);
	int32_t matches_size = matches.size();
	if (matches_size < 20)
	{
		if (++lost_count_ >= 10)
		{
			std::cout << "Tracking Lost!" << std::endl;
			tracking_state_ = LOST;
		}
		return false;
	}

	cur_inliers_ = OptimizePose(cur_frame_, last_frame_, matches);

	if (cur_inliers_ <= 10)
	{
		if (++lost_count_ >= 10)
		{
			std::cout << "Tracking Lost!" << std::endl;
			tracking_state_ = LOST;
		}
		return false;
	}

	std::cout << "Tracking With RefFrame Succeeded!" << std::endl;
	return true;
}

bool Tracking::Relocalization()
{
	int32_t key_frame_size = key_frames_.size();

	for (int32_t i = key_frame_size - 1; i >= 0; i--)
	{
		std::vector<cv::DMatch> matches = MatchTwoFrame(cur_frame_, key_frames_[i]);
		int32_t matches_size = matches.size();
		if (matches_size < 20)
		{
			continue;
		}

		cur_inliers_ = OptimizePose(cur_frame_, last_frame_, matches);

		if (cur_inliers_ <= 10)
		{
			continue;
		}
		else
		{
			std::cout << "Tracking Relocalization Succeeded!" << std::endl;
			lost_count_ = 0;
			tracking_state_ = OK;
			return true;
		}
	}

	std::cout << "Tracking Relocalization Failed!" << std::endl;
	return false;
}

std::vector<cv::DMatch> Tracking::MatchTwoFrame(const Frame * p_query_frame, const Frame * p_train_frame)
{
	cv::BruteForceMatcher<cv::HammingLUT> matcher;
	std::vector<std::vector<cv::DMatch>> matches_knn;
	std::vector<cv::DMatch> matches;

	matcher.knnMatch(p_query_frame->desciptors_, p_train_frame->desciptors_, matches_knn, 2);

	for (int32_t i = 0, size = matches_knn.size(); i < size; i++)
	{
		if (matches_knn[i][0].distance < match_ratio_ * matches_knn[i][1].distance)
		{
			matches.push_back(matches_knn[i][0]);
		}
	}

	return matches;
}

int32_t Tracking::OptimizePose(const Frame * p_query_frame, Frame * p_train_frame, std::vector<cv::DMatch> & p_matches)
{
	float camera_fx = camera_K_.at<float>(0, 0);
	float camera_fy = camera_K_.at<float>(1, 1);
	float camera_cx = camera_K_.at<float>(0, 2);
	float camera_cy = camera_K_.at<float>(1, 2);

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;
	int32_t matches_size = p_matches.size();

	for (int32_t i = 0; i < matches_size; i++)
	{
		cv::Point2f point_2f = p_query_frame->key_points_[p_matches[i].queryIdx].pt;
		uint16_t depth = p_train_frame->depth_image_.ptr<uint16_t>((int32_t)point_2f.y)[(int32_t)point_2f.x];

		if (depth == 0) continue;

		train_frame_points.push_back(cv::Point2f(p_train_frame->key_points_[p_matches[i].trainIdx].pt));

		cv::Point3f point_3f(point_2f.x, point_2f.y, (float)depth);

		cv::Point3f world_point;
		world_point.z = point_3f.z / camera_scale_;
		world_point.x = (point_3f.x - camera_cx) * world_point.z / camera_fx;
		world_point.y = (point_3f.y - camera_cy) * world_point.z / camera_fy;

		query_frame_points.push_back(world_point);
	}

	if ((query_frame_points.size() == 0) || (train_frame_points.size() == 0))
	{
		return 0;
	}

	cv::Mat rotation, translation, inliers;
	cv::solvePnPRansac(query_frame_points, train_frame_points, camera_K_, camera_D_, rotation, translation, false, 100, 8.0f, 100, inliers);

	cv::Mat rotation_3x3;
	Rodrigues(rotation, rotation_3x3);
	Eigen::Matrix3d rotation_eigen;
	cv::cv2eigen(rotation_3x3, rotation_eigen);
	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	Eigen::AngleAxisd angle(rotation_eigen);
	transform = angle;
	transform(0, 3) = translation.at<double>(0, 0);
	transform(1, 3) = translation.at<double>(1, 0);
	transform(2, 3) = translation.at<double>(2, 0);
	p_train_frame->SetTransform(transform);

	// std::cout << "Transform Matrix:" << std::endl << transform.matrix() << std::endl;
	std::cout << "Inliers number: " << inliers.rows << std::endl;

	return inliers.rows;
}
