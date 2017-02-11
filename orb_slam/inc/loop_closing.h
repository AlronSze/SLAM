#pragma once

#include <g2o/core/sparse_optimizer.h>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "parameter.h"
#include "frame.h"
#include "map.h"

class LoopClosing
{
public:
	LoopClosing(const Parameter & p_parameter, Map * p_map);

	void GetKeyFrame(const Frame & p_frame);
	void OptimizeLast();
	inline void SaveG2OFile(const char * p_g2o_file_name);
	inline void PopKeyFrame();

private:
	void SetBowVector(Frame & p_frame);
	void LoadVocabulary();
	void InitializeG2O();
	void AddCurFrameToGraph();
	void LoopClose();
	std::vector<Frame *> GetLoopFrames();
	int32_t GetPose(const Frame & p_query_frame, const Frame & p_train_frame, Eigen::Isometry3d & p_transform);

private:
	std::vector<Frame> key_frames_;
	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> vocabulary_;
	Frame cur_frame_;
	g2o::SparseOptimizer optimizer_;
	std::string vocabulary_dir_;
	double dbow2_score_min_;
	int32_t dbow2_interval_min_;
	cv::Mat camera_K_;
	cv::Mat camera_D_;
	float match_ratio_;
	int32_t match_threshold_;
	int32_t pnp_inliers_threshold_;
	double local_error_sum_;
	double global_error_sum_;
	double chi2_threshold_;
	Map * map_;
	int32_t frames_count_;
};

inline void LoopClosing::SaveG2OFile(const char * p_g2o_file_name)
{
	optimizer_.save(p_g2o_file_name);
}

inline void LoopClosing::PopKeyFrame()
{
	key_frames_.pop_back();
}
