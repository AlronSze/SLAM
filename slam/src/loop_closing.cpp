#include "../inc/loop_closing.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

LoopClosing::LoopClosing(const Parameter &p_parameter, DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary,
	                     Map *p_map, QLineEdit *p_value_loop_count) :
	local_error_sum_(0.0), global_error_sum_(0.0), not_optimize_count_(0), bow_vocabulary_(p_bow_vocabulary), map_(p_map),
	value_loop_count_(p_value_loop_count), global_loop_count_(0)
{
	cv::Mat camera_k = cv::Mat::eye(3, 3, CV_32F);
	camera_k.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	camera_k.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	camera_k.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	camera_k.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;

	bow_score_min_ = p_parameter.kDBoW2ScoreMin_;
	bow_interval_min_ = p_parameter.kDBoW2IntervalMin_;
	chi2_threshold_ = p_parameter.kG2OChi2Threshold_;

	pnp_solver_ = new PnPSolver(p_parameter.KPNPInliersThreshold_, p_parameter.kORBMatchRatio_, camera_k);

	InitializeG2O();
}

LoopClosing::~LoopClosing()
{
	delete pnp_solver_;
}

void LoopClosing::InitializeG2O()
{
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> *linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	//linear_solver->setBlockOrdering(false);
	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer_.setAlgorithm(algorithm);
	optimizer_.setVerbose(false);
}

void LoopClosing::GetKeyFrame(const Frame &p_frame)
{
	current_frame_ = Frame(p_frame);
	current_frame_.SetBowVector(*bow_vocabulary_);

	if (key_frames_.size() != 0)
	{
		AddCurrentFrameToGraph();
		LoopClose();
		key_frames_.push_back(current_frame_);
	}
	else
	{
		key_frames_.push_back(current_frame_);
		g2o::VertexSE3 *vertex = new g2o::VertexSE3();
		vertex->setId(current_frame_.id_);
		vertex->setEstimate(Eigen::Isometry3d::Identity());
		vertex->setFixed(true);
		optimizer_.addVertex(vertex);

		map_->GetKeyFrames(key_frames_);
	}

	//SaveG2OFile("tracking.g2o");
}

void LoopClosing::OptimizeLast()
{
	std::cout << "Last Optimizing..." << std::endl;

	optimizer_.initializeOptimization();
	optimizer_.optimize(10);

	for (size_t i = 0, for_size = key_frames_.size(); i < for_size; ++i)
	{
		g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frames_[i].id_));
		key_frames_[i].SetTransformWorldToCamera(vertex->estimate());
	}

	std::cout << "Optimized!" << std::endl;

	map_->GetKeyFrames(key_frames_, true);
}

void LoopClosing::AddCurrentFrameToGraph()
{
	g2o::VertexSE3 *vertex = new g2o::VertexSE3();
	vertex->setId(current_frame_.id_);
	vertex->setEstimate(current_frame_.transform_world_to_camera_);
	vertex->setFixed(false);
	optimizer_.addVertex(vertex);

	g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
	edge->vertices()[0] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(current_frame_.id_));
	edge->vertices()[1] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frames_.back().id_));
	edge->setMeasurement(current_frame_.transform_camera_to_last_);
	edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
	edge->setRobustKernel(new g2o::RobustKernelHuber());
	optimizer_.addEdge(edge);
}

void LoopClosing::DetectLocalLoop()
{
	int32_t key_frames_size = (int32_t)key_frames_.size();

	std::cout << "Detecting local loop closure..." << std::endl;
	const int32_t start_index = key_frames_size - 2;
	const int32_t end_index = ((start_index - 10 + 1) >= 0) ? (start_index - 10 + 1) : 0;
	pnp_solver_->ResetInliersThreshold(80);

	#pragma omp parallel for
	for (int32_t i = start_index; i >= end_index; --i)
	{
		const Frame &key_frame = key_frames_[i];
		Eigen::Isometry3d transform_estimate = key_frame.transform_camera_to_world_ * current_frame_.transform_world_to_camera_;
		pnp_solver_->SetTransformEstimate(transform_estimate);

		if (!pnp_solver_->SolvePnP(current_frame_, key_frame))
		{
			continue;
		}

		g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frame.id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(current_frame_.id_));
		edge->setMeasurement(pnp_solver_->GetTransform());
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		edge->computeError();

		#pragma omp critical (local_loop_section)
		{
			local_error_sum_ += edge->chi2();
			std::cout << "Local loop closure with " << key_frame.id_ << ", error: " << edge->chi2()
				<< ", total error: " << local_error_sum_ << std::endl;
			optimizer_.addEdge(edge);
		}
	}
}

void LoopClosing::DetectGlobalLoop()
{
	std::cout << "Detecting global loop closure..." << std::endl;
	const std::vector<int32_t> loop_frames_index = GetLoopFrames();
	const int32_t loop_frames_size = (int32_t)loop_frames_index.size();
	pnp_solver_->ResetInliersThreshold(40);

	#pragma omp parallel for
	for (int32_t i = 0; i < loop_frames_size; ++i)
	{
		const Frame &key_frame = key_frames_[loop_frames_index[i]];
		Eigen::Isometry3d transform_estimate = key_frame.transform_camera_to_world_ * current_frame_.transform_world_to_camera_;
		pnp_solver_->SetTransformEstimate(transform_estimate);

		if (!pnp_solver_->SolvePnP(current_frame_, key_frame))
		{
			continue;
		}

		++global_loop_count_;

		g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frame.id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(current_frame_.id_));
		edge->setMeasurement(pnp_solver_->GetTransform());
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		edge->computeError();

		#pragma omp critical (global_loop_section)
		{
			global_error_sum_ += edge->chi2();
			std::cout << "Global loop closure with " << key_frame.id_ << ", error: " << edge->chi2()
				<< ", total error: " << global_error_sum_ << std::endl;
			optimizer_.addEdge(edge);
		}
	}
}

void LoopClosing::LoopClose()
{
	DetectLocalLoop();
	DetectGlobalLoop();

	if ((global_error_sum_ > chi2_threshold_) || (local_error_sum_ > chi2_threshold_) || ((++not_optimize_count_) >= 10))
	{
		std::cout << "Optimizing..." << std::endl;

		optimizer_.initializeOptimization();
		optimizer_.optimize(10);

		for (size_t i = 0, for_size = key_frames_.size(); i < for_size; ++i)
		{
			g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frames_[i].id_));
			key_frames_[i].SetTransformWorldToCamera(vertex->estimate());
		}

		global_error_sum_ = 0.0;
		local_error_sum_ = 0.0;
		not_optimize_count_ = 0;

		std::cout << "Optimized!" << std::endl;

		map_->GetKeyFrames(key_frames_);
	}

	value_loop_count_->setText(QString().setNum(global_loop_count_));
}

std::vector<int32_t> LoopClosing::GetLoopFrames()
{
	std::vector<int32_t> result;

	#pragma omp parallel for
	for (int32_t i = (int32_t)key_frames_.size() - 2 - 10; i >= 0; --i)
	{
		double score = bow_vocabulary_->score(current_frame_.bow_vector_, key_frames_[i].bow_vector_);
		// std::cout << "DBoW2 score with " << key_frames_[i].id_ << " : " << score << std::endl;
		if ((score > bow_score_min_) && ((current_frame_.id_ - key_frames_[i].id_) > bow_interval_min_))
		{
			#pragma omp critical (bow_loop_section)
			{
				result.push_back(i);
			}
		}
	}

	return result;
}
