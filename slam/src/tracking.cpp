#include "../inc/tracking.h"

#include <iostream>

#include <opencv2/core/eigen.hpp>

#include "../inc/orb_matcher.h"
#include "../inc/pnp_solver.h"

#ifndef SAFE_DELETE 
#define SAFE_DELETE(p) if(p) { delete (p); (p) = NULL; }
#endif

Tracking::Tracking(const Parameter &p_parameter, DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary, LoopClosing *p_loop_closing, QLineEdit *p_value_keyframe_count) :
	tracking_state_(INITIALIZE), loop_closing_(p_loop_closing), last_transform_(Eigen::Isometry3d::Identity()),
	key_frames_count_(0), value_keyframe_count_(p_value_keyframe_count), bow_vocabulary_(p_bow_vocabulary), debug_time_(0)
{
	cv::Mat camera_k = cv::Mat::eye(3, 3, CV_32F);
	camera_k.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	camera_k.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	camera_k.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	camera_k.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;

	bow_score_min_ = p_parameter.kDBoW2TrackScore_;
	match_track_ratio_ = p_parameter.kMatchTrackRatio_;
	match_relocalize_ratio_ = p_parameter.kMatchRelocalizeRatio_;
	pnp_track_threshold_ = p_parameter.KPNPTrackThreshold_;
	pnp_relocalize_threshold_ = p_parameter.KPNPRelocalizeThreshold_;

	keyframe_norm_min_ = p_parameter.kKeyframeNormMin_;
	keyframe_norm_max_ = p_parameter.kKeyframeNormMax_;

	pnp_solver_ = new PnPSolver(p_parameter.KPNPTrackThreshold_, p_parameter.kMatchTrackRatio_, camera_k);
}

Tracking::~Tracking()
{
	SAFE_DELETE(pnp_solver_);
}

bool Tracking::GetFrame(Frame &p_frame)
{
	clock_t s, e;
	s = clock();

	bool track_result;
	current_frame_ = p_frame;
	track_result = Track();

	e = clock();
	debug_time_ += e - s;

	return track_result;
}

bool Tracking::Track()
{
	bool is_tracked = true;
	bool is_relocalized = false;

	if (tracking_state_ == INITIALIZE)
	{
		if (Initialization())
		{
			// std::cout << "Insert New Key Frame, number: 1" << std::endl;
			current_frame_.SetPointCloud();
			current_frame_.ReleaseImage();
			current_frame_.SetBowVector(*bow_vocabulary_);
			UpdateAndLoop();
			return true;
		}
	}
	else if (tracking_state_ == OK)
	{
		is_tracked = TrackWithLastKeyFrame();

		if (!is_tracked)
		{
			is_tracked = Relocalization();
			is_relocalized = is_tracked;
		}
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
			current_frame_.SetPointCloud();
			current_frame_.ReleaseImage();
			UpdateAndLoop();
			value_keyframe_count_->setText(QString().setNum(key_frames_count_));
			// std::cout << "Insert New Key Frame, number: " << key_frames_count_ << std::endl;
		}
	}

	if (tracking_state_ == LOST)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Tracking::Initialization()
{
	if (current_frame_.key_point_number_ >= 500)
	{
		// std::cout << "Tracking Initialized!" << std::endl;
		tracking_state_ = OK;
		return true;
	}

	// std::cout << "Tracking Initialization Failed!" << std::endl;
	return false;
}

bool Tracking::TrackWithLastKeyFrame()
{
	pnp_solver_->SetTransformEstimate(last_transform_);
	pnp_solver_->ResetMatchRatio(match_track_ratio_);
	pnp_solver_->ResetInliersThreshold(pnp_track_threshold_);

	if (pnp_solver_->SolvePnP(key_frames_.back(), current_frame_))
	{
		// std::cout << "Tracking With Last Key Frame Succeeded!" << std::endl;
		UpdateFrameTransform();
		return true;
	}

	// std::cout << "Tracking Lost!" << std::endl;
	tracking_state_ = LOST;
	return false;
}

bool Tracking::Relocalization()
{
	// std::cout << "Tracking Relocalizing..." << std::endl;
	pnp_solver_->SetTransformEstimate(last_transform_);
	pnp_solver_->ResetMatchRatio(match_relocalize_ratio_);
	pnp_solver_->ResetInliersThreshold(pnp_relocalize_threshold_);

	//const int32_t start_index = key_frames_count_ - 1;
	//const int32_t end_index = ((start_index - 10) > 0) ? (start_index - 10) : 0;
	//for (int32_t i = start_index, delete_frames_count = 0; i >= end_index; --i)
	//{
	//	if (!pnp_solver_->SolvePnP(key_frames_[i], current_frame_))
	//	{
	//		++delete_frames_count;
	//		continue;
	//	}

	//	for (int32_t j = 0; j < delete_frames_count; ++j)
	//	{
	//		key_frames_.pop_back();
	//		--key_frames_count_;
	//		loop_closing_->PopKeyFrame();
	//	}

	//	// std::cout << "Tracking Relocalization Succeeded!" << std::endl;
	//	UpdateFrameTransform();
	//	tracking_state_ = OK;
	//	return true;
	//}

	current_frame_.SetBowVector(*bow_vocabulary_);
	const std::vector<int32_t> bow_frames_index = GetBoWFrames();
	const int32_t bow_frames_size = (int32_t)bow_frames_index.size();

	for (int32_t i = 0; i < bow_frames_size; ++i)
	{
		if (!pnp_solver_->SolvePnP(key_frames_[bow_frames_index[i]], current_frame_))
		{
			continue;
		}

		const int32_t delete_frames_count = key_frames_count_ - bow_frames_index[i] - 1;

		for (int32_t j = 0; j < delete_frames_count; ++j)
		{
			key_frames_.pop_back();
			--key_frames_count_;
			loop_closing_->PopKeyFrame();
		}

		// std::cout << "Tracking Relocalization Succeeded!" << std::endl;
		UpdateFrameTransform();
		tracking_state_ = OK;
		return true;
	}

	// std::cout << "Tracking Relocalization Failed!" << std::endl;
	return false;
}

bool Tracking::NeedInsertKeyFrame(const bool p_flag)
{
	if (p_flag)
	{
		return true;
	}

	double norm = fabs(cv::norm(current_translation_)) + fabs(std::min(cv::norm(current_rotation_), 2.0 * M_PI - cv::norm(current_rotation_)));
	bool norm_flag = (norm < keyframe_norm_max_) && (norm > keyframe_norm_min_);

	return norm_flag;
}

void Tracking::UpdateAndLoop()
{
	last_transform_ = Eigen::Isometry3d::Identity();
	key_frames_.push_back(current_frame_);
	++key_frames_count_;
	loop_closing_->GetKeyFrame(current_frame_);
}

void Tracking::UpdateFrameTransform()
{
	last_transform_ = pnp_solver_->GetTransform();
	current_frame_.SetTransformCameraToLast(last_transform_);
	current_frame_.SetTransformCameraToWorld(last_transform_ * key_frames_.back().transform_camera_to_world_);

	Eigen::Matrix3d rotation = last_transform_.rotation();
	Eigen::Vector3d translation(last_transform_(0, 3), last_transform_(1, 3), last_transform_(2, 3));
	cv::eigen2cv(rotation, current_rotation_);
	cv::eigen2cv(translation, current_translation_);
}

std::vector<int32_t> Tracking::GetBoWFrames()
{
	std::vector<int32_t> result;

	for (int32_t i = (int32_t)key_frames_.size() - 1; i >= 0; --i)
	{
		double score = bow_vocabulary_->score(current_frame_.bow_vector_, key_frames_[i].bow_vector_);
		// std::cout << "DBoW2 score with " << key_frames_[i].id_ << " : " << score << std::endl;
		if (score >= bow_score_min_)
		{
			result.push_back(i);
		}
	}

	return result;
}
