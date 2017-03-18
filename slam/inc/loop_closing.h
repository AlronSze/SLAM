#pragma once

#include <QLineEdit>

#include <g2o/core/sparse_optimizer.h>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "frame.h"
#include "map.h"
#include "parameter.h"
#include "pnp_solver.h"

class LoopClosing
{
public:
	LoopClosing(const Parameter &p_parameter, DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary,
		        Map *p_map, QLineEdit *p_value_loop_count);
	~LoopClosing();

	void GetKeyFrame(const Frame &p_frame);
	void OptimizeLast();
	void SaveG2OFile(const char *p_g2o_file_name);
	void PopKeyFrame();

private:
	void InitializeG2O();
	void AddCurrentFrameToGraph();
	void DetectLocalLoop();
	void DetectGlobalLoop();
	void LoopClose();
	std::vector<int32_t> GetLoopFrames();

private:
	Frame current_frame_;
	std::vector<Frame> key_frames_;

	int32_t global_loop_count_;

	g2o::SparseOptimizer optimizer_;

	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *bow_vocabulary_;
	double bow_score_min_;
	int32_t bow_interval_min_;

	double local_error_sum_;
	double global_error_sum_;
	int32_t not_optimize_count_;
	double chi2_threshold_;

	QLineEdit *value_loop_count_;

	Map *map_;
	PnPSolver *pnp_solver_;
};

inline void LoopClosing::SaveG2OFile(const char *p_g2o_file_name)
{
	optimizer_.save(p_g2o_file_name);
}

inline void LoopClosing::PopKeyFrame()
{
	key_frames_.pop_back();
}
