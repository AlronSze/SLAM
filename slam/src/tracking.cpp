#include "../inc/tracking.h"

#include <iostream>

#include <opencv2/core/eigen.hpp>

#include "../inc/orb_matcher.h"
#include "../inc/pnp_solver.h"

Tracking::Tracking(const Parameter &p_parameter, LoopClosing *p_loop_closing, QLineEdit *p_value_keyframe_count) :
	tracking_state_(INITIALIZE), loop_closing_(p_loop_closing), last_transform_(Eigen::Isometry3d::Identity()),
	key_frames_count_(0), value_keyframe_count_(p_value_keyframe_count)
{
	cv::Mat camera_k = cv::Mat::eye(3, 3, CV_32F);
	camera_k.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	camera_k.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	camera_k.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	camera_k.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;

	pnp_solver_ = new PnPSolver(p_parameter.KPNPInliersThreshold_, p_parameter.kORBMatchRatio_, camera_k);
}

Tracking::~Tracking()
{
	delete pnp_solver_;
}

bool Tracking::GetFrame(Frame &p_frame)
{
	bool track_result;
	current_frame_ = p_frame;
	track_result = Track();
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
			std::cout << "Insert New Key Frame, number: 1" << std::endl;
			current_frame_.SetPointCloud();
			current_frame_.ReleaseImage();
			UpdateAndLoop();
			return true;
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
			current_frame_.SetPointCloud();
			current_frame_.ReleaseImage();
			UpdateAndLoop();
			value_keyframe_count_->setText(QString().setNum(key_frames_count_));
			std::cout << "Insert New Key Frame, number: " << key_frames_count_ << std::endl;
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
		std::cout << "Tracking Initialized!" << std::endl;
		tracking_state_ = OK;
		return true;
	}

	std::cout << "Tracking Initialization Failed!" << std::endl;
	return false;
}

bool Tracking::TrackWithLastKeyFrame()
{
	pnp_solver_->SetTransformEstimate(last_transform_);
	pnp_solver_->ResetMatchRatio(0.7);
	pnp_solver_->ResetInliersThreshold(80);

	if (pnp_solver_->SolvePnP(key_frames_.back(), current_frame_))
	{
		std::cout << "Tracking With Last Key Frame Succeeded!" << std::endl;
		UpdateFrameTransform();
		return true;
	}

	std::cout << "Tracking Lost!" << std::endl;
	tracking_state_ = LOST;
	return false;
}

bool Tracking::Relocalization()
{
	std::cout << "Tracking Relocalizing..." << std::endl;
	pnp_solver_->SetTransformEstimate(last_transform_);
	pnp_solver_->ResetMatchRatio(0.7);
	pnp_solver_->ResetInliersThreshold(40);

	const int32_t start_index = key_frames_count_ - 1;
	const int32_t end_index = ((start_index - 10) > 0) ? (start_index - 10) : 0;
	for (int32_t i = start_index, delete_frames_count = 0; i >= end_index; --i)
	{
		if (!pnp_solver_->SolvePnP(key_frames_[i], current_frame_))
		{
			++delete_frames_count;
			continue;
		}

		for (int32_t j = 0; j < delete_frames_count; ++j)
		{
			key_frames_.pop_back();
			--key_frames_count_;
			loop_closing_->PopKeyFrame();
		}

		std::cout << "Tracking Relocalization Succeeded!" << std::endl;
		UpdateFrameTransform();
		tracking_state_ = OK;
		return true;
	}

	std::cout << "Tracking Relocalization Failed!" << std::endl;
	return false;
}

bool Tracking::NeedInsertKeyFrame(const bool p_flag)
{
	if (p_flag)
	{
		return true;
	}

	double norm = fabs(cv::norm(current_translation_)) + fabs(std::min(cv::norm(current_rotation_), 2.0 * M_PI - cv::norm(current_rotation_)));
	bool norm_flag = (norm < 1.86) && (norm > 1.76);

	return norm_flag;
}

void Tracking::UpdateAndLoop()
{
	//last_transform_ = Eigen::Isometry3d::Identity();
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
