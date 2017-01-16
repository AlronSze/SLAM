#pragma once

#include <deque>
#include <opencv2/core/core.hpp>

#include "parameter.h"
#include "frame.h"

class Tracking {
public:
	Tracking(const Parameter & p_parameter);

	void GetFrame(Frame * p_frame);

private:
	void Track();
	bool Initialization();
	bool TrackWithMotion();
	bool TrackWithRefFrame();
	bool Relocalization();
	std::vector<cv::DMatch> MatchTwoFrame(const Frame * p_query_frame, const Frame * p_train_frame);
	int32_t OptimizePose(const Frame * p_query_frame, Frame * p_train_frame, std::vector<cv::DMatch> & p_matches);

private:
	enum TrackingState {
		INITIALIZE = 0,
		OK,
		LOST
	} tracking_state_;

	float match_ratio_;
	float camera_scale_;
	int32_t lost_count_;
	int32_t cur_inliers_;
	int32_t ref_inliers_;
	cv::Mat camera_K_;
	cv::Mat camera_D_;
	Frame *cur_frame_;
	Frame *last_frame_;
	Frame *ref_frames_;
	std::deque<Frame *> key_frames_;
};
