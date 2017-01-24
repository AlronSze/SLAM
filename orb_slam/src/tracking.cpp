#include "../inc/tracking.h"

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/optimizer.h"

Tracking::Tracking(const Parameter & p_parameter, LoopClosing * p_loop_closing) :
	tracking_state_(INITIALIZE), loop_closing_(p_loop_closing), last_key_frame_dist_(0), 
	last_transform_(Eigen::Isometry3d::Identity())
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

	match_ratio_ = p_parameter.kORBMatchRatio_;
	pnp_inliers_threshold_ = p_parameter.KPNPInliersThreshold_;
}

void Tracking::GetFrame(Frame * p_frame)
{
	cur_frame_ = p_frame;
	Track();
}

void Tracking::Track()
{
	bool is_tracked = true;
	bool is_relocalized = false;

	if (tracking_state_ == INITIALIZE)
	{
		if (Initialization())
		{
			key_frames_.push_back(Frame(*cur_frame_));
			loop_closing_->GetKeyFrame(Frame(*cur_frame_));
		}

		last_frame_ = Frame(*cur_frame_);
		return;
	}
	else if (tracking_state_ == OK)
	{
		//is_tracked = TrackWithMotion();
		//if (!is_tracked)
		//{
			is_tracked = TrackWithLastKeyFrame();
		//}
	}
	else
	{
		is_tracked = Relocalization();
		is_relocalized = is_tracked;
	}

	if (is_tracked)
	{
		double norm = fabs(cv::norm(cur_translation_)) + fabs(std::min(cv::norm(cur_rotation_), 2.0 * M_PI - cv::norm(cur_rotation_)));
		//std::cout << "Frame norm: " << norm << std::endl;
		if (((norm < 1.9) && (norm > 1.78)) || is_relocalized)
		{
			std::cout << "Insert New Key Frame, number: " << key_frames_.size() + 1 << std::endl;
			key_frames_.push_back(Frame(*cur_frame_));
			loop_closing_->GetKeyFrame(Frame(*cur_frame_));
			last_key_frame_dist_ = 0;
		}
	}

	last_key_frame_dist_++;
	last_frame_ = Frame(*cur_frame_);
}

bool Tracking::Initialization()
{
	if (cur_frame_->key_point_number_ >= 500)
	{
		std::cout << "Tracking Initialized!" << std::endl;
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
	if (OptimizePose(last_frame_, *cur_frame_) < pnp_inliers_threshold_)
	{
		return false;
	}

	std::cout << "Tracking With Motion Succeeded!" << std::endl;
	return true;
}

bool Tracking::TrackWithLastKeyFrame()
{
	if (OptimizePose(key_frames_.back(), *cur_frame_) < pnp_inliers_threshold_)
	{
		std::cout << "Tracking Lost!" << std::endl;
		tracking_state_ = LOST;
		return false;
	}

	std::cout << "Tracking With Last Key Frame Succeeded!" << std::endl;
	return true;
}

bool Tracking::Relocalization()
{
	std::cout << "Tracking Relocalizing..." << std::endl;

	int32_t key_frame_size = (int32_t)key_frames_.size();
	int32_t delete_frames_count = 0;
	for (int32_t i = key_frame_size - 1; (i >= 0) && (i >= key_frame_size - 1 - 10); i--)
	{
		if (OptimizePose(key_frames_[i], *cur_frame_) < 10)
		{
			delete_frames_count++;
			continue;
		}

		std::cout << "Tracking Relocalization Succeeded!" << std::endl;
		tracking_state_ = OK;

		for (; delete_frames_count > 0; delete_frames_count--)
		{
			key_frames_.pop_back();
			loop_closing_->PopKeyFrame();
		}

		return true;
	}

	std::cout << "Tracking Relocalization Failed!" << std::endl;
	return false;
}

std::vector<cv::DMatch> Tracking::MatchTwoFrame(const Frame & p_query_frame, const Frame & p_train_frame)
{
	cv::BruteForceMatcher<cv::HammingLUT> matcher;
	std::vector<std::vector<cv::DMatch>> matches_knn;
	std::vector<cv::DMatch> matches;

	matcher.knnMatch(p_query_frame.descriptors_, p_train_frame.descriptors_, matches_knn, 2);

	for (int32_t i = 0, size = (int32_t)matches_knn.size(); i < size; i++)
	{
		if (matches_knn[i][0].distance < match_ratio_ * matches_knn[i][1].distance)
		{
			matches.push_back(matches_knn[i][0]);
		}
	}

	return matches;
}

int32_t Tracking::OptimizePose(const Frame & p_query_frame, Frame & p_train_frame)
{
	std::vector<cv::DMatch> matches = MatchTwoFrame(p_query_frame, p_train_frame);
	int32_t matches_size = (int32_t)matches.size();
	//std::cout << "Matches number: " << matches_size << std::endl;
	if (matches_size < pnp_inliers_threshold_)
	{
		return 0;
	}

	float camera_fx = camera_K_.at<float>(0, 0);
	float camera_fy = camera_K_.at<float>(1, 1);
	float camera_cx = camera_K_.at<float>(0, 2);
	float camera_cy = camera_K_.at<float>(1, 2);

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;

	for (int32_t i = 0; i < matches_size; i++)
	{
		uint16_t depth = p_query_frame.point_depth_[matches[i].queryIdx];
		if (depth == 0) continue;

		query_frame_points.push_back(cv::Point3f(p_query_frame.point_3d_[matches[i].queryIdx]));
		train_frame_points.push_back(cv::Point2f(p_train_frame.key_points_[matches[i].trainIdx].pt));
	}

	if ((query_frame_points.size() == 0) || (train_frame_points.size() == 0))
	{
		return 0;
	}

	std::vector<int32_t> inliers_index;
	Optimizer::PnPSolver(query_frame_points, train_frame_points, camera_K_, inliers_index, last_transform_);
	p_train_frame.SetTransform(last_transform_);

	Eigen::Matrix3d rotation = last_transform_.rotation();
	Eigen::Vector3d translation(last_transform_(0, 3), last_transform_(1, 3), last_transform_(2, 3));
	cv::eigen2cv(rotation, cur_rotation_);
	cv::eigen2cv(translation, cur_translation_);
	//std::cout << "Inliers Number: " << inliers_index.size() << std::endl;

	 return inliers_index.size();
}
