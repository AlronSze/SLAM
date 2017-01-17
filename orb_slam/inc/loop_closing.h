#pragma once

#include <g2o/core/sparse_optimizer.h>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "parameter.h"
#include "frame.h"
#include "tracking.h"

class LoopClosing {
public:
	LoopClosing(const Parameter & p_parameter);

	void GetKeyFrame(const Frame & p_frame);
	void SetBowVector(Frame & p_frame);
	inline void GetTracking(Tracking * p_tracking);

private:
	void AddCurFrameToGraph();
	void LoopClose();

private:
	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> vocabulary_;
	std::vector<Frame> key_frames_;
	Frame cur_frame_;
	Tracking * tracking_;
	g2o::SparseOptimizer optimizer;
};

inline void LoopClosing::GetTracking(Tracking * p_tracking)
{
	tracking_ = p_tracking;
}
