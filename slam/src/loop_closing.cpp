#include "../inc/loop_closing.h"

#ifdef _WIN32
	#include <windows.h>
	#define thread_sleep(x) Sleep(x)
#elif __linux__
	#include <unistd.h>
	#define thread_sleep(x) usleep(x)
#endif

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/optimizer.h"
#include "../inc/orb_matcher.h"

LoopClosing::LoopClosing(const Parameter & p_parameter, DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> * p_bow_vocabulary,
	Map * p_map, QLineEdit * p_value_loop_count) :
	local_error_sum_(0.0), global_error_sum_(0.0), frames_count_(0), bow_vocabulary_(p_bow_vocabulary), map_(p_map),
	value_loop_count_(p_value_loop_count), global_loop_count_(0)
{
	cv::Mat temp_K = cv::Mat::eye(3, 3, CV_32F);
	temp_K.at<float>(0, 0) = p_parameter.kCameraParameters_.fx_;
	temp_K.at<float>(1, 1) = p_parameter.kCameraParameters_.fy_;
	temp_K.at<float>(0, 2) = p_parameter.kCameraParameters_.cx_;
	temp_K.at<float>(1, 2) = p_parameter.kCameraParameters_.cy_;
	temp_K.copyTo(camera_K_);

	dbow2_score_min_ = p_parameter.kDBoW2ScoreMin_;
	dbow2_interval_min_ = p_parameter.kDBoW2IntervalMin_;
	match_ratio_ = p_parameter.kORBMatchRatio_;
	pnp_inliers_threshold_ = p_parameter.KPNPInliersThreshold_;
	chi2_threshold_ = p_parameter.kG2OChi2Threshold_;

	InitializeG2O();
}

void LoopClosing::InitializeG2O()
{
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> * linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	// linear_solver->setBlockOrdering(false);
	g2o::BlockSolver_6_3 * block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg * algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer_.setAlgorithm(algorithm);
	optimizer_.setVerbose(false);
}

void LoopClosing::GetKeyFrame(const Frame & p_frame)
{
	cur_frame_ = Frame(p_frame);
	SetBowVector(cur_frame_);
	++frames_count_;

	if (key_frames_.size() != 0)
	{
		AddCurFrameToGraph();
		LoopClose();
		key_frames_.push_back(cur_frame_);

		//if (frames_count_ >= 10)
		//{
		//	frames_count_ = 0;
		//	map_->GetKeyFrames(key_frames_);
		//}
	}
	else
	{
		key_frames_.push_back(cur_frame_);
		g2o::VertexSE3* vertex = new g2o::VertexSE3();
		vertex->setId(cur_frame_.id_);
		vertex->setEstimate(Eigen::Isometry3d::Identity());
		vertex->setFixed(true);
		optimizer_.addVertex(vertex);

		map_->GetKeyFrames(key_frames_);
	}

	SaveG2OFile("tracking.g2o");
}

void LoopClosing::OptimizeLast()
{
	std::vector<Frame> optimized_key_frames;

	std::cout << "Last Optimizing..." << std::endl;
	optimizer_.initializeOptimization();
	optimizer_.optimize(10);

	for (size_t i = 0, for_size = key_frames_.size(); i < for_size; ++i)
	{
		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
		key_frames_[i].SetTransform(vertex->estimate());
		optimized_key_frames.push_back(key_frames_[i]);
	}

	std::cout << "Optimized!" << std::endl;

	std::cout << "Global Bundle Adjustment..." << std::endl;
	Optimizer::BundleAdjustment(optimized_key_frames, 20);
	std::cout << "Global Bundle Adjustment Finished!" << std::endl;

	map_->GetKeyFrames(optimized_key_frames, true);
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

void LoopClosing::ModifyMapPoints(const int32_t p_frame_id, const std::vector<cv::DMatch> & p_matches, const std::vector<int8_t> & p_matches_flag)
{
	Frame & key_frame = key_frames_[p_frame_id];
	std::vector<int8_t> train_flag(key_frame.key_point_number_, 0);
	const int32_t matches_size = (int32_t)p_matches.size();
	const int32_t insert_id = cur_frame_.id_;

	#pragma omp parallel for
	for (int32_t i = 0; i < matches_size; ++i)
	{
		if (p_matches_flag[i])
		{
			const int32_t query_index = p_matches[i].queryIdx;
			const int32_t train_index = p_matches[i].trainIdx;

			MapPoint *const query_map_point = cur_frame_.map_points_[query_index];
			MapPoint *const train_map_point = key_frame.map_points_[train_index];

			if (query_map_point->is_bad_ || train_map_point->is_bad_)
			{
				continue;
			}

			#pragma omp critical (section2)
			{
				if (!train_flag[train_index])
				{
					train_flag[train_index] = 1;

					for (auto observation : query_map_point->observation_id_)
					{
						train_map_point->InsertObservation(observation.first, observation.second);
					}

					for (auto observation : train_map_point->observation_id_)
					{
						query_map_point->InsertObservation(observation.first, observation.second);
					}

					//train_map_point->InsertObservation(insert_id, query_index);
					train_map_point->point_2d_ = query_map_point->point_2d_;
					train_map_point->point_3d_ = query_map_point->point_3d_;
					train_map_point->rgb_r_ = query_map_point->rgb_r_;
					train_map_point->rgb_g_ = query_map_point->rgb_g_;
					train_map_point->rgb_b_ = query_map_point->rgb_b_;

					//cur_frame_.map_points_[query_index] = train_map_point;
				}
			}
		}
	}
}

void LoopClosing::LoopClose()
{
	int32_t key_frames_size = (int32_t)key_frames_.size();

	std::cout << "Local Bundle Adjustment..." << std::endl;
	const int32_t ba_start_index = ((key_frames_size - 5) >= 0) ? (key_frames_size - 5) : 0;
	Optimizer::BundleAdjustment(key_frames_, 10, ba_start_index);

	std::cout << "Detecting local loop closure..." << std::endl;
	const int32_t start_index = key_frames_size - 2;
	const int32_t end_index = ((start_index - 10 + 1) >= 0) ? (start_index - 10 + 1) : 0;

	#pragma omp parallel for
	for (int32_t i = start_index; i >= end_index; --i)
	{
		Eigen::Isometry3d transform = cur_frame_.GetTransform();
		std::vector<cv::DMatch> matches;
		std::vector<int8_t> matches_flag;
		if (!GetPose(cur_frame_, key_frames_[i], transform, matches, matches_flag, 100))
		{
			continue;
		}

		g2o::EdgeSE3 * edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(key_frames_[i].id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3 *> (optimizer_.vertex(cur_frame_.id_));
		edge->setMeasurement(transform);
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		edge->computeError();

		#pragma omp critical (section)
		{
			ModifyMapPoints(i, matches, matches_flag);
			local_error_sum_ += edge->chi2();
			std::cout << "Local loop closure with " << key_frames_[i].id_ << ", error: " << edge->chi2()
				<< ", total error: " << local_error_sum_ << std::endl;
			optimizer_.addEdge(edge);
		}
	}

	std::cout << "Detecting global loop closure..." << std::endl;
	std::vector<int32_t> loop_frames_index = GetLoopFrames();
	const int32_t loop_frames_size = (int32_t)loop_frames_index.size();

	#pragma omp parallel for
	for (int32_t i = 0; i < loop_frames_size; ++i)
	{
		const int32_t loop_frame_index = loop_frames_index[i];
		Eigen::Isometry3d transform = cur_frame_.GetTransform();
		std::vector<cv::DMatch> matches;
		std::vector<int8_t> matches_flag;
		if (!GetPose(cur_frame_, key_frames_[loop_frame_index], transform, matches, matches_flag, 40))
		{
			continue;
		}

		++global_loop_count_;

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[loop_frame_index].id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
		edge->setMeasurement(transform);
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		edge->computeError();

		#pragma omp critical (section)
		{
			ModifyMapPoints(loop_frame_index, matches, matches_flag);
			global_error_sum_ += edge->chi2();
			std::cout << "Global loop closure with " << loop_frame_index << ", error: " << edge->chi2()
				<< ", total error: " << global_error_sum_ << std::endl;
			optimizer_.addEdge(edge);
		}
	}

	std::vector<Frame> optimized_key_frames;
	if ((global_error_sum_ > chi2_threshold_) || (local_error_sum_ > chi2_threshold_) || (frames_count_ >= 10))
	{
		std::cout << "Optimizing..." << std::endl;
		optimizer_.initializeOptimization();
		optimizer_.optimize(10);

		for (size_t i = 0; i < (size_t)key_frames_size; ++i)
		{
			g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
			key_frames_[i].SetTransform(vertex->estimate());
			optimized_key_frames.push_back(key_frames_[i]);
		}

		global_error_sum_ = 0.0;
		local_error_sum_ = 0.0;
		frames_count_ = 0;
		std::cout << "Optimized!" << std::endl;

		map_->GetKeyFrames(optimized_key_frames);
	}

	value_loop_count_->setText(QString().setNum(global_loop_count_));
}

std::vector<int32_t> LoopClosing::GetLoopFrames()
{
	std::vector<int32_t> result;

	#pragma omp parallel for
	for (int32_t i = (int32_t)key_frames_.size() - 2 - 10; i >= 0; --i)
	{
		double score = bow_vocabulary_->score(cur_frame_.bow_vector, key_frames_[i].bow_vector);
		// std::cout << "DBoW2 score with " << key_frames_[i].id_ << " : " << score << std::endl;
		if ((score > dbow2_score_min_) && ((cur_frame_.id_ - key_frames_[i].id_) > dbow2_interval_min_))
		{
			#pragma omp critical (section)
			{
				result.push_back(i);
			}
		}
	}

	return result;
}

void LoopClosing::SetBowVector(Frame & p_frame)
{
	DBoW2::FeatureVector feature_vector;
	bow_vocabulary_->transform(p_frame.GetDescriptorVector(), p_frame.bow_vector, feature_vector, 4);
}

bool LoopClosing::GetPose(const Frame & p_query_frame, const Frame & p_train_frame, Eigen::Isometry3d & p_transform,
	std::vector<cv::DMatch> & p_matches, std::vector<int8_t> & p_matches_flag, const int32_t p_threshold)
{
	p_matches = ORBMatcher::MatchTwoFrame(p_query_frame, p_train_frame, match_ratio_);
	int32_t matches_size = (int32_t)p_matches.size();
	// std::cout << "Match Number: " << matches_size << std::endl;

	if (matches_size == 0)
	{
		return false;
	}

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;
	query_frame_points.reserve(matches_size);
	train_frame_points.reserve(matches_size);

	std::vector<int32_t> matches_valid;
	matches_valid.reserve(matches_size);

	for (int32_t i = 0; i < matches_size; ++i)
	{
		uint16_t depth = p_query_frame.point_depth_[p_matches[i].queryIdx];
		if (depth == 0)
		{
			continue;
		}

		query_frame_points.push_back(cv::Point3f(p_query_frame.point_3d_[p_matches[i].queryIdx]));
		train_frame_points.push_back(cv::Point2f(p_train_frame.point_2d_[p_matches[i].trainIdx]));
		matches_valid.push_back(i);
	}

	int32_t matches_valid_size = (int32_t)matches_valid.size();
	if (matches_valid_size == 0)
	{
		return false;
	}

	std::vector<int8_t> inliers_mask(matches_valid_size, 1);
	int32_t inliers_number = Optimizer::PnPSolver(query_frame_points, train_frame_points, camera_K_, inliers_mask, p_transform);
	// std::cout << "Inliers Number: " << inliers_number << std::endl;

	if (inliers_number < p_threshold)
	{
		return false;
	}

	p_matches_flag.resize(matches_size, 0);

	#pragma omp parallel for
	for (int32_t i = 0; i < matches_valid_size; ++i)
	{
		if (inliers_mask[i])
		{
			p_matches_flag[matches_valid[i]] = 1;
		}
	}

	return true;
}
