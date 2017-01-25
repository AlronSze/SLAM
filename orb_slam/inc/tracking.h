#pragma once

#include <deque>
#include <opencv2/core/core.hpp>

#include "parameter.h"
#include "frame.h"
#include "loop_closing.h"

class Tracking {
public:
	Tracking(const Parameter & p_parameter, LoopClosing * p_loop_closing);

	void GetFrame(Frame * p_frame);

private:
	void Track();
	bool Initialization();
	bool TrackWithMotion();
	bool TrackWithLastKeyFrame();
	bool Relocalization();
	int32_t OptimizePose(const Frame & p_query_frame, Frame & p_train_frame);

private:
	enum TrackingState {
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
	LoopClosing *loop_closing_;
	int32_t last_key_frame_dist_;
	Eigen::Isometry3d last_transform_;
};
