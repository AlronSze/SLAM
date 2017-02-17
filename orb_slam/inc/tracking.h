#pragma once

#include <deque>
#include <opencv2/core/core.hpp>

#include "parameter.h"
#include "frame.h"
#include "loop_closing.h"

class Tracking
{
public:
	Tracking(const Parameter & p_parameter, LoopClosing * p_loop_closing);
	~Tracking();

	void GetFrame(Frame * p_frame);

private:
	void Track();
	bool Initialization();
	bool TrackWithLastKeyFrame();
	bool Relocalization();
	bool NeedInsertKeyFrame(const bool p_flag);
	void ModifyMapPoints();
	void UpdateKeyFrames();
	bool OptimizePose(const Frame & p_query_frame, Frame & p_train_frame, const int32_t p_threshold);

private:
	enum TrackingState
	{
		INITIALIZE = 0,
		OK,
		LOST
	} tracking_state_;

	float match_ratio_;
	int32_t match_threshold_;
	int32_t pnp_inliers_threshold_;

	cv::Mat camera_K_;
	cv::Mat camera_D_;

	cv::Mat cur_rotation_;
	cv::Mat cur_translation_;

	Frame *cur_frame_;
	Frame last_frame_;

	std::vector<Frame> key_frames_;
	std::vector<cv::DMatch> cur_matches_;
	std::vector<int8_t> cur_matches_flag_;
	int32_t cur_matches_size_;

	LoopClosing *loop_closing_;

	Eigen::Isometry3d last_transform_;
};

inline bool Tracking::NeedInsertKeyFrame(const bool p_flag)
{
	double norm = fabs(cv::norm(cur_translation_)) + fabs(std::min(cv::norm(cur_rotation_), 2.0 * M_PI - cv::norm(cur_rotation_)));
	bool norm_flag = (norm < 1.9) && (norm > 1.78);
	return (norm_flag || p_flag);
}

inline void Tracking::UpdateKeyFrames()
{
	key_frames_.push_back(*cur_frame_);
	loop_closing_->GetKeyFrame(*cur_frame_);
	last_frame_ = *cur_frame_;
}
