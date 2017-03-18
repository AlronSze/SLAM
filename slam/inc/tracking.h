#pragma once

#include <QLineEdit>

#include <vector>

#include <opencv2/core/core.hpp>

#include "frame.h"
#include "loop_closing.h"
#include "parameter.h"
#include "pnp_solver.h"

class Tracking
{
public:
	Tracking(const Parameter &p_parameter, LoopClosing *p_loop_closing, QLineEdit *p_value_keyframe_count);
	~Tracking();

	bool GetFrame(Frame &p_frame);

private:
	bool Track();
	bool Initialization();
	bool TrackWithLastKeyFrame();
	bool Relocalization();
	bool NeedInsertKeyFrame(const bool p_flag);
	void UpdateAndLoop();
	void UpdateFrameTransform();

private:
	enum TrackingState
	{
		INITIALIZE = 0,
		OK,
		LOST
	} tracking_state_;

	cv::Mat current_rotation_;
	cv::Mat current_translation_;

	Frame current_frame_;
	std::vector<Frame> key_frames_;
	int32_t key_frames_count_;

	QLineEdit *value_keyframe_count_;

	LoopClosing *loop_closing_;
	PnPSolver *pnp_solver_;

	Eigen::Isometry3d last_transform_;
};
