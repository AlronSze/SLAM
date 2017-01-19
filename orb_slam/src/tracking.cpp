#include "../inc/tracking.h"

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

Tracking::Tracking(const Parameter & p_parameter, LoopClosing * p_loop_closing) :
	tracking_state_(INITIALIZE), cur_inliers_(0), loop_closing_(p_loop_closing), last_key_frame_dist_(0), 
	speed_(Eigen::Isometry3d::Identity()), relocalization_count_(0)
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
	pnp_iterations_count_ = p_parameter.kPNPIterationsCount_;
	pnp_error_ = p_parameter.kPNPError_;
	pnp_min_inliers_count_ = p_parameter.kPNPMinInliersCount_;
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
	bool is_track_with_ref = false;
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
			is_tracked = TrackWithRefFrame();
			//is_track_with_ref = is_tracked;
		//}
	}
	else
	{
		is_tracked = Relocalization();
		is_relocalized = is_tracked;
	}

	if (is_tracked)
	{
		// TO DO: SHOULD FIX CONDITIONS
		double norm = fabs(cv::norm(cur_translation_)) + fabs(std::min(cv::norm(cur_rotation_), 2.0 * M_PI - cv::norm(cur_rotation_)));
		// std::cout << norm << std::endl;
		//if (((norm < 0.5) && (norm > 0.1)) || (is_track_with_ref && ((cur_frame_->id_ - key_frames_.back().id_) > 10)) || is_relocalized || (last_key_frame_dist_ >= 30))
		if (((norm < 0.2) && (norm > 0.1)) || is_relocalized || (last_key_frame_dist_ >= 30))
		{
			std::cout << "Insert New Key Frame, number: " << key_frames_.size() + 1 << std::endl;
			key_frames_.push_back(Frame(*cur_frame_));
			loop_closing_->GetKeyFrame(Frame(*cur_frame_));
			last_key_frame_dist_ = 0;
			loop_closing_->SaveG2OFile("tracking.g2o");
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
	cur_inliers_ = OptimizePose(last_frame_, *cur_frame_);
	if (cur_inliers_ < pnp_inliers_threshold_)
	{
		return false;
	}

	std::cout << "Tracking With Motion Succeeded!" << std::endl;
	return true;
}

bool Tracking::TrackWithRefFrame()
{
	cur_inliers_ = OptimizePose(key_frames_.back(), *cur_frame_);
	if (cur_inliers_ < pnp_inliers_threshold_)
	{
		std::cout << "Tracking Lost!" << std::endl;
		tracking_state_ = LOST;
		return false;
	}

	std::cout << "Tracking With RefFrame Succeeded!" << std::endl;
	return true;
}

bool Tracking::Relocalization()
{
	std::cout << "Tracking Relocalizing..." << std::endl;

	int32_t key_frame_size = (int32_t)key_frames_.size();
	int32_t delete_frames_count = 0;
	for (int32_t i = key_frame_size - 1; (i >= 0) && (i >= key_frame_size - 1 - 10); i--)
	{
		cur_inliers_ = OptimizePose(key_frames_[i], *cur_frame_);
		if (cur_inliers_ < pnp_inliers_threshold_)
		{
			delete_frames_count++;
			continue;
		}

		std::cout << "Tracking Relocalization Succeeded!" << std::endl;
		tracking_state_ = OK;

		for (; delete_frames_count > 0; delete_frames_count--)
		{
			key_frames_.pop_back();
			loop_closing_->key_frames_.pop_back();
		}

		relocalization_count_ = 0;
		return true;
	}

	if (++relocalization_count_ >= 4)
	{
		std::cout << "Tracking Relocalization by Last Key Frame." << std::endl;
		relocalization_count_ = 0;
		cur_frame_->SetTransform(key_frames_.back().GetTransform());
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
	//std::cout << "PnP number: " << query_frame_points.size() << std::endl;

	if ((query_frame_points.size() == 0) || (train_frame_points.size() == 0))
	{
		return 0;
	}

	cv::Mat inliers;
	cv::solvePnPRansac(query_frame_points, train_frame_points, camera_K_, camera_D_,
		cur_rotation_, cur_translation_, false, pnp_iterations_count_, pnp_error_, pnp_min_inliers_count_, inliers);

	cv::Mat rotation_3x3;
	Rodrigues(cur_rotation_, rotation_3x3);
	Eigen::Matrix3d rotation_eigen;
	cv::cv2eigen(rotation_3x3, rotation_eigen);
	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	Eigen::AngleAxisd angle(rotation_eigen);
	transform = angle;
	transform(0, 3) = cur_translation_.at<double>(0, 0);
	transform(1, 3) = cur_translation_.at<double>(1, 0);
	transform(2, 3) = cur_translation_.at<double>(2, 0);
	p_train_frame.SetTransform(transform);
	//std::cout << "Inliers number: " << inliers.rows << std::endl;

	return inliers.rows;
}
