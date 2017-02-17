#include "../inc/tracking.h"

#include <iostream>
#include <opencv2/core/eigen.hpp>

#include "../inc/optimizer.h"
#include "../inc/map_point.h"

Tracking::Tracking(const Parameter & p_parameter, LoopClosing * p_loop_closing) :
	tracking_state_(INITIALIZE), loop_closing_(p_loop_closing), last_transform_(Eigen::Isometry3d::Identity())
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
	match_threshold_ = p_parameter.kORBMatchThreshold_;
	pnp_inliers_threshold_ = p_parameter.KPNPInliersThreshold_;
}

Tracking::~Tracking()
{
	for (size_t i = 0, for_size_1 = key_frames_.size(); i < for_size_1; ++i)
	{
		Frame & key_frame = key_frames_[i];
		const int32_t frame_id = key_frame.id_;
		for (size_t j = 0, for_size_2 = key_frame.map_points_.size(); j < for_size_2; ++j)
		{
			if ((key_frame.map_points_[j]->is_bad_) || (key_frame.map_points_[j]->best_id_ == frame_id))
			{
				delete key_frame.map_points_[j];
			}
		}
	}
}

void Tracking::GetFrame(Frame * p_frame)
{
	cur_frame_ = p_frame;
	Track();
}

void Tracking::ModifyMapPoints()
{
	std::vector<int8_t> train_flag(cur_frame_->key_point_number_, 0);
	Frame cur_frame_temp = *cur_frame_;
	Frame & last_key_frame = key_frames_.back();

	#pragma omp parallel for
	for (int32_t i = 0; i < cur_matches_size_; ++i)
	{
		if (cur_matches_flag_[i])
		{
			const int32_t query_index = cur_matches_[i].queryIdx;
			const int32_t train_index = cur_matches_[i].trainIdx;

			MapPoint *const query_map_point = last_key_frame.map_points_[query_index];
			MapPoint *const train_map_point = cur_frame_temp.map_points_[train_index];

			if (query_map_point->is_bad_ || train_map_point->is_bad_)
			{
				continue;
			}

			query_map_point->InsertObservation(cur_frame_->id_);
			query_map_point->point_2d_ = train_map_point->point_2d_;
			query_map_point->point_3d_ = train_map_point->point_3d_;
			query_map_point->rgb_r_ = train_map_point->rgb_r_;
			query_map_point->rgb_g_ = train_map_point->rgb_g_;
			query_map_point->rgb_b_ = train_map_point->rgb_b_;

			#pragma omp critical (section)
			{
				if (!train_flag[train_index])
				{
					train_flag[train_index] = 1;
					delete cur_frame_->map_points_[train_index];
					cur_frame_->map_points_[train_index] = query_map_point;
				}
			}
		}
	}
}

void Tracking::Track()
{
	bool is_tracked = true;
	bool is_relocalized = false;

	if (tracking_state_ == INITIALIZE)
	{
		if (Initialization())
		{
			std::cout << "Insert New Key Frame, number: 1" << std::endl;
			cur_frame_->InitializeMapPoints();
			cur_frame_->ReleaseImage();
			UpdateKeyFrames();
		}
	}
	else if (tracking_state_ == OK)
	{
		is_tracked = TrackWithLastKeyFrame();
	}
	else
	{
		is_tracked = Relocalization();
		is_relocalized = is_tracked;
	}

	if (is_tracked)
	{
		if (NeedInsertKeyFrame(is_relocalized))
		{
			std::cout << "Insert New Key Frame, number: " << key_frames_.size() + 1 << std::endl;
			cur_frame_->InitializeMapPoints();
			cur_frame_->ReleaseImage();
			ModifyMapPoints();
			UpdateKeyFrames();
		}
	}
}

bool Tracking::Initialization()
{
	if (cur_frame_->key_point_number_ >= 500)
	{
		std::cout << "Tracking Initialized!" << std::endl;
		tracking_state_ = OK;
		return true;
	}

	std::cout << "Tracking Initialization Failed!" << std::endl;
	return false;
}

bool Tracking::TrackWithLastKeyFrame()
{
	if (OptimizePose(key_frames_.back(), *cur_frame_, pnp_inliers_threshold_))
	{
		std::cout << "Tracking With Last Key Frame Succeeded!" << std::endl;
		return true;
	}

	std::cout << "Tracking Lost!" << std::endl;
	tracking_state_ = LOST;
	return false;
}

bool Tracking::Relocalization()
{
	std::cout << "Tracking Relocalizing..." << std::endl;

	const size_t start_index = key_frames_.size() - 1;
	const size_t end_index = (((int32_t)start_index - 10) > 0) ? (start_index - 10) : 0;
	for (size_t i = start_index, delete_frames_count = 0; i >= end_index; --i)
	{
		if (!OptimizePose(key_frames_[i], *cur_frame_, 10))
		{
			++delete_frames_count;
			continue;
		}

		std::cout << "Tracking Relocalization Succeeded!" << std::endl;
		tracking_state_ = OK;

		for (int32_t j = 0; j < delete_frames_count; ++j)
		{
			Frame & key_frame = key_frames_[(int32_t)start_index - j];
			int32_t map_points_size = (int32_t)key_frame.map_points_.size();

			#pragma omp parallel for
			for (int32_t k = 0; k < map_points_size; ++k)
			{
				MapPoint * map_point = key_frame.map_points_[k];

				if (!map_point->is_bad_)
				{
					if (map_point->observation_count_ <= 1)
					{
						delete map_point;
					}
					else
					{
						map_point->EraseObservation(key_frame.id_);
					}
				}
			}

			key_frames_.pop_back();
			loop_closing_->PopKeyFrame();
		}

		return true;
	}

	std::cout << "Tracking Relocalization Failed!" << std::endl;
	return false;
}

bool Tracking::OptimizePose(const Frame & p_query_frame, Frame & p_train_frame, const int32_t p_threshold)
{
	cur_matches_ = Frame::MatchTwoFrame(p_query_frame, p_train_frame, match_ratio_);
	cur_matches_size_ = (int32_t)cur_matches_.size();
	// std::cout << "Match Number: " << cur_matches_size_ << std::endl;
	
	if (cur_matches_size_ < match_threshold_)
	{
		return false;
	}

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;
	query_frame_points.reserve(cur_matches_size_);
	train_frame_points.reserve(cur_matches_size_);

	std::vector<int32_t> matches_valid;
	matches_valid.reserve(cur_matches_size_);
	
	for (int32_t i = 0; i < cur_matches_size_; ++i)
	{
		uint16_t depth = p_query_frame.point_depth_[cur_matches_[i].queryIdx];
		if (depth == 0)
		{
			continue;
		}

		query_frame_points.push_back(cv::Point3f(p_query_frame.point_3d_[cur_matches_[i].queryIdx]));
		train_frame_points.push_back(cv::Point2f(p_train_frame.key_points_[cur_matches_[i].trainIdx].pt));
		matches_valid.push_back(i);
	}

	int32_t matches_valid_size = (int32_t)matches_valid.size();
	if (matches_valid_size == 0)
	{
		return false;
	}

	std::vector<int8_t> inliers_mask(matches_valid_size, 1);
	int32_t inliers_number = Optimizer::PnPSolver(query_frame_points, train_frame_points, camera_K_, inliers_mask, last_transform_);
	// std::cout << "Inliers Number: " << inliers_number << std::endl;
	
	if (inliers_number < p_threshold)
	{
		return false;
	}

	cur_matches_flag_.resize(cur_matches_size_, 0);

	#pragma omp parallel for
	for (int32_t i = 0; i < matches_valid_size; ++i)
	{
		if (inliers_mask[i])
		{
			cur_matches_flag_[matches_valid[i]] = 1;
		}
	}

	p_train_frame.SetTransform(last_transform_);
	Eigen::Matrix3d rotation = last_transform_.rotation();
	Eigen::Vector3d translation(last_transform_(0, 3), last_transform_(1, 3), last_transform_(2, 3));
	cv::eigen2cv(rotation, cur_rotation_);
	cv::eigen2cv(translation, cur_translation_);

	return true;
}
