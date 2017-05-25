#pragma once

#include <QLineEdit>

#include <vector>

#include <opencv2/core/core.hpp>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "frame.h"
#include "loop_closing.h"
#include "parameter.h"
#include "pnp_solver.h"

class Tracking
{
public:
	Tracking(const Parameter &p_parameter, DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary, LoopClosing *p_loop_closing, QLineEdit *p_value_keyframe_count);
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
	std::vector<int32_t> GetBoWFrames();

public:
	clock_t debug_time_;

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

	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *bow_vocabulary_;
	double bow_score_min_;

	QLineEdit *value_keyframe_count_;

	LoopClosing *loop_closing_;
	PnPSolver *pnp_solver_;

	float match_track_ratio_;
	float match_relocalize_ratio_;
	int32_t pnp_track_threshold_;
	int32_t pnp_relocalize_threshold_;

	double keyframe_norm_min_;
	double keyframe_norm_max_;

	Eigen::Isometry3d last_transform_;
};
