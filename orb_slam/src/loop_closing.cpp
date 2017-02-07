#include "../inc/loop_closing.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/optimizer.h"

LoopClosing::LoopClosing(const Parameter & p_parameter, Map * p_map) :
	local_error_sum_(0.0), global_error_sum_(0.0), frames_count_(0)
{
	cv::Mat temp_K = cv::Mat::eye(3, 3, CV_32F);
	temp_K.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	temp_K.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	temp_K.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	temp_K.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;
	temp_K.copyTo(camera_K_);

	cv::Mat temp_D(5, 1, CV_32F);
	temp_D.at<float>(0) = p_parameter.kCameraParameters_.d0_;
	temp_D.at<float>(1) = p_parameter.kCameraParameters_.d1_;
	temp_D.at<float>(2) = p_parameter.kCameraParameters_.d2_;
	temp_D.at<float>(3) = p_parameter.kCameraParameters_.d3_;
	temp_D.at<float>(4) = p_parameter.kCameraParameters_.d4_;
	temp_D.copyTo(camera_D_);

	vocabulary_dir_ = p_parameter.kVocabularyDir_;
	dbow2_score_min_ = p_parameter.kDBoW2ScoreMin_;
	dbow2_interval_min_ = p_parameter.kDBoW2IntervalMin_;
	match_ratio_ = p_parameter.kORBMatchRatio_;
	match_threshold_ = p_parameter.kORBMatchThreshold_;
	pnp_inliers_threshold_ = p_parameter.KPNPInliersThreshold_;
	chi2_threshold_ = p_parameter.kG2OChi2Threshold_;

	map_ = p_map;

	if (!vocabulary_dir_.empty())
	{
		LoadVocabulary();
	}
	InitializeG2O();
}

void LoopClosing::LoadVocabulary()
{
	std::cout << "Loading vocabulary file..." << std::endl;
	vocabulary_.loadFromTextFile(vocabulary_dir_);
	std::cout << "Vocabulary Loaded!" << std::endl;
}

void LoopClosing::InitializeG2O()
{
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>* linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	// linear_solver->setBlockOrdering(false);
	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer_.setAlgorithm(algorithm);
	optimizer_.setVerbose(false);
}

void LoopClosing::GetKeyFrame(const Frame & p_frame)
{
	cur_frame_ = Frame(p_frame);
	SetBowVector(cur_frame_);
	frames_count_++;

	if (key_frames_.size() != 0)
	{
		AddCurFrameToGraph();
		LoopClose();
		key_frames_.push_back(cur_frame_);
	}
	else
	{
		key_frames_.push_back(cur_frame_);
		g2o::VertexSE3* vertex = new g2o::VertexSE3();
		vertex->setId(cur_frame_.id_);
		vertex->setEstimate(Eigen::Isometry3d::Identity());
		vertex->setFixed(true);
		optimizer_.addVertex(vertex);
	}

	SaveG2OFile("tracking.g2o");
}

void LoopClosing::OptimizeLast()
{
	std::vector<Frame> optimized_key_frames;

	std::cout << "Last Optimizing..." << std::endl;
	optimizer_.initializeOptimization();
	optimizer_.optimize(10);

	//for (auto key_frame : key_frames_)
	//{
	//	g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frame.id_));
	//	key_frame.SetTransform(vertex->estimate());
	//	optimized_key_frames.push_back(key_frame);
	//}
	int32_t key_frames_size = (int32_t)key_frames_.size();
	for (int32_t i = 0; i < key_frames_size; i++)
	{
		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
		key_frames_[i].SetTransform(vertex->estimate());
		optimized_key_frames.push_back(key_frames_[i]);
	}

	std::cout << "Optimized!" << std::endl;

	while (!map_->can_draw_);
	map_->GetKeyFrames(optimized_key_frames);
}

void LoopClosing::AddCurFrameToGraph()
{
	g2o::VertexSE3* vertex = new g2o::VertexSE3();
	vertex->setId(cur_frame_.id_);
	vertex->setEstimate(Eigen::Isometry3d::Identity());
	vertex->setFixed(false);
	optimizer_.addVertex(vertex);

	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
	edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_.back().id_));
	edge->setMeasurement(cur_frame_.GetTransform());
	edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
	edge->setRobustKernel(new g2o::RobustKernelHuber());
	optimizer_.addEdge(edge);
}

void LoopClosing::LoopClose()
{
	std::cout << "Detecting local loop closure..." << std::endl;

	int32_t frames_number = (int32_t)key_frames_.size();
	int32_t end_index = ((frames_number - 2 - 10 + 1) >= 0) ? (frames_number - 2 - 10 + 1) : 0;

	#pragma omp parallel for
	for (int32_t i = frames_number - 2; i >= end_index; i--)
	{
		Eigen::Isometry3d transform = cur_frame_.GetTransform();
		if (GetPose(cur_frame_, key_frames_[i], transform) < 40)
		{
			continue;
		}

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
		edge->setMeasurement(transform);
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		edge->computeError();

		#pragma omp critical (section)
		{
			local_error_sum_ += edge->chi2();
			std::cout << "Local loop closure with " << key_frames_[i].id_ << ", error: " << edge->chi2()
				<< ", total error: " << local_error_sum_ << std::endl;
			optimizer_.addEdge(edge);
		}
	}

	std::cout << "Detecting global loop closure..." << std::endl;
	std::vector<Frame *> loop_frames = GetLoopFrames();
	int32_t loop_frames_size = (int32_t)loop_frames.size();
	if (loop_frames_size != 0)
	{
		#pragma omp parallel for
		for (int32_t i = 0; i < loop_frames_size; i++)
		{
			Eigen::Isometry3d transform = cur_frame_.GetTransform();
			if (GetPose(cur_frame_, *loop_frames[i], transform) < 40)
			{
				continue;
			}

			g2o::EdgeSE3* edge = new g2o::EdgeSE3();
			edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(loop_frames[i]->id_));
			edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
			edge->setMeasurement(transform);
			edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
			edge->setRobustKernel(new g2o::RobustKernelHuber());
			edge->computeError();

			#pragma omp critical (section)
			{
				global_error_sum_ += edge->chi2();
				std::cout << "Global loop closure with " << loop_frames[i]->id_ << ", error: " << edge->chi2()
					<< ", total error: " << global_error_sum_ << std::endl;
				optimizer_.addEdge(edge);
			}
		}
	}

	bool is_optimized = false;
	std::vector<Frame> optimized_key_frames;
	if ((global_error_sum_ > chi2_threshold_) || (local_error_sum_ > chi2_threshold_) || (frames_count_ >= 10))
	{
		std::cout << "Optimizing..." << std::endl;
		optimizer_.initializeOptimization();
		optimizer_.optimize(10);

		//for (auto key_frame : key_frames_)
		//{
		//	g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frame.id_));
		//	key_frame.SetTransform(vertex->estimate());
		//	optimized_key_frames.push_back(key_frame);
		//}
		int32_t key_frames_size = (int32_t)key_frames_.size();
		for (int32_t i = 0; i < key_frames_size; i++)
		{
			g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
			key_frames_[i].SetTransform(vertex->estimate());
			optimized_key_frames.push_back(key_frames_[i]);
		}

		global_error_sum_ = 0.0;
		local_error_sum_ = 0.0;
		frames_count_ = 0;
		is_optimized = true;
		std::cout << "Optimized!" << std::endl;
	}

	if (is_optimized)
	{
		while (!map_->can_draw_);
		map_->GetKeyFrames(optimized_key_frames);
	}
}

std::vector<Frame *> LoopClosing::GetLoopFrames()
{
	std::vector<Frame *> result;
	int32_t frames_number = (int32_t)key_frames_.size();

	#pragma omp parallel for
	for (int32_t i = frames_number - 2 - 10; i >= 0; i--)
	{
		double score = vocabulary_.score(cur_frame_.bow_vector, key_frames_[i].bow_vector);
		// std::cout << "DBoW2 score with " << key_frames_[i].id_ << " : " << score << std::endl;
		if ((score > dbow2_score_min_) && ((cur_frame_.id_ - key_frames_[i].id_) > dbow2_interval_min_))
		{
			#pragma omp critical (section)
			{
				result.push_back(&key_frames_[i]);
			}
		}
	}

	return result;
}

void LoopClosing::SetBowVector(Frame & p_frame)
{
	DBoW2::FeatureVector feature_vector;
	vocabulary_.transform(p_frame.GetDescriptorVector(), p_frame.bow_vector, feature_vector, 2);
}

int32_t LoopClosing::GetPose(const Frame & p_query_frame, const Frame & p_train_frame, Eigen::Isometry3d & p_transform)
{
	std::vector<cv::DMatch> matches = Frame::MatchTwoFrame(p_query_frame, p_train_frame, match_ratio_);
	int32_t matches_size = (int32_t)matches.size();

	if (matches_size < match_threshold_) return 0;

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;
	std::vector<int32_t> match_valid_index;
	query_frame_points.reserve(matches_size);
	train_frame_points.reserve(matches_size);
	match_valid_index.reserve(matches_size);

	for (int32_t i = 0; i < matches_size; i++)
	{
		uint16_t depth = p_query_frame.point_depth_[matches[i].queryIdx];
		if (depth == 0) continue;

		query_frame_points.push_back(cv::Point3f(p_query_frame.point_3d_[matches[i].queryIdx]));
		train_frame_points.push_back(cv::Point2f(p_train_frame.key_points_[matches[i].trainIdx].pt));
		match_valid_index.push_back(i);
	}

	if (query_frame_points.empty()) return 0;
	
	std::vector<bool> inliers_mask(match_valid_index.size(), true);
	int32_t inliers_number = Optimizer::PnPSolver(query_frame_points, train_frame_points, camera_K_, inliers_mask, p_transform);

	return inliers_number;
}
