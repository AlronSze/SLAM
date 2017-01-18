#include "../inc/loop_closing.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

LoopClosing::LoopClosing(const Parameter & p_parameter) : local_error_sum_(0.0), global_error_sum_(0.0)
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
	dbow2_score_min_ = p_parameter.kDBoW2ScoreMin;
	dbow2_interval_min_ = p_parameter.kDBoW2IntervalMin;
	match_ratio_ = p_parameter.kMatchRatio_;
	pnp_iterations_count_ = p_parameter.kPNPIterationsCount_;
	pnp_error_ = p_parameter.kPNPError_;
	pnp_min_inliers_count_ = p_parameter.kPNPMinInliersCount_;
	pnp_inliers_threshold_ = p_parameter.KPNPInliersThreshold_;

	// LoadVocabulary();
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
	linear_solver->setBlockOrdering(false);
	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer_.setAlgorithm(algorithm);
	optimizer_.setVerbose(false);
}

void LoopClosing::GetKeyFrame(const Frame & p_frame)
{
	cur_frame_ = Frame(p_frame);
	SetBowVector(cur_frame_);

	if (key_frames_.size() != 0)
	{
		AddCurFrameToGraph();
		// LoopClose();
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
}

void LoopClosing::AddCurFrameToGraph()
{
	g2o::VertexSE3* vertex = new g2o::VertexSE3();
	vertex->setId(cur_frame_.id_);
	vertex->setEstimate(Eigen::Isometry3d::Identity());
	vertex->setFixed(false);
	optimizer_.addVertex(vertex);

	g2o::EdgeSE3* edge = new g2o::EdgeSE3();

	//g2o::VertexSE3* vertex_0 = dynamic_cast<g2o::VertexSE3 *>(optimizer_.vertex(key_frames_.back().id_));
	//g2o::VertexSE3* vertex_1 = dynamic_cast<g2o::VertexSE3 *>(optimizer_.vertex(cur_frame_.id_));
	//edge->setVertex(1, vertex_0);
	//edge->setVertex(0, vertex_1);
	//edge->setMeasurementFromState();
	edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_.back().id_));
	edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
	edge->setMeasurement(cur_frame_.GetTransform());

	edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
	edge->setRobustKernel(new g2o::RobustKernelHuber());
	optimizer_.addEdge(edge);
}

void LoopClosing::LoopClose()
{
	int32_t frames_number = (int32_t)key_frames_.size();

	std::cout << "Detecting local loop closure..." << std::endl;
	for (int32_t i = frames_number - 2; (i >= 0) && (i > (frames_number - 2 - 5)); i--)
	{
		Eigen::Isometry3d transform;
		if (GetPose(key_frames_[i], cur_frame_, transform) < pnp_inliers_threshold_)
		{
			continue;
		}

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(key_frames_[i].id_));
		edge->setMeasurement(transform);
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		optimizer_.addEdge(edge);
		edge->computeError();
		std::cout << "Local loop closure with " << key_frames_[i].id_ << ", error: " << edge->chi2();
		local_error_sum_ += edge->chi2();
		std::cout << ", total error: " << local_error_sum_ << std::endl;
	}

	std::cout << "Detecting global loop closure..." << std::endl;
	std::vector<Frame *> loop_frames = GetLoopFrames();
	int32_t loop_frames_size = loop_frames.size();
	if (loop_frames_size == 0)
	{
		return;
	}
	for (auto loop_frame : loop_frames)
	{
		Eigen::Isometry3d transform;
		if (GetPose(*loop_frame, cur_frame_, transform) < pnp_inliers_threshold_)
		{
			continue;
		}

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(cur_frame_.id_));
		edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer_.vertex(loop_frame->id_));
		edge->setMeasurement(transform);
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		optimizer_.addEdge(edge);
		edge->computeError();
		std::cout << "Global loop closure with " << loop_frame->id_ << ", error: " << edge->chi2();
		global_error_sum_ += edge->chi2();
		std::cout << ", total error: " << global_error_sum_ << std::endl;
	}
}

std::vector<Frame *> LoopClosing::GetLoopFrames()
{
	std::vector<Frame *> result;
	int32_t frames_number = (int32_t)key_frames_.size();
	for (int32_t i = frames_number - 2; i >= 0; i--)
	{
		double score = vocabulary_.score(cur_frame_.bow_vector, key_frames_[i].bow_vector);
		// std::cout << "DBoW2 score with " << key_frames_[i].id_ << " : " << score << std::endl;
		if ((score > dbow2_score_min_) && ((cur_frame_.id_ - key_frames_[i].id_) > dbow2_interval_min_))
		{
			result.push_back(&key_frames_[i]);
		}
	}
	return result;
}

void LoopClosing::SetBowVector(Frame & p_frame)
{
	DBoW2::FeatureVector feature_vector;
	vocabulary_.transform(p_frame.GetDescriptorVector(), p_frame.bow_vector, feature_vector, 2);
}

std::vector<cv::DMatch> LoopClosing::MatchTwoFrame(const Frame & p_query_frame, const Frame & p_train_frame)
{
	cv::BruteForceMatcher<cv::HammingLUT> matcher;
	std::vector<std::vector<cv::DMatch>> matches_knn;
	std::vector<cv::DMatch> matches;

	matcher.knnMatch(p_query_frame.descriptors_, p_train_frame.descriptors_, matches_knn, 2);

	for (int32_t i = 0, size = (int32_t)matches_knn.size(); i < size; i++)
	{
		if (matches_knn[i][0].distance < match_ratio_ * matches_knn[i][1].distance)
		{
			matches.push_back(matches_knn[i][0]);
		}
	}

	return matches;
}

int32_t LoopClosing::GetPose(const Frame & p_query_frame, const Frame & p_train_frame, Eigen::Isometry3d & p_transform)
{
	std::vector<cv::DMatch> matches = MatchTwoFrame(p_query_frame, p_train_frame);
	int32_t matches_size = (int32_t)matches.size();

	if (matches_size < 20)
	{
		return 0;
	}

	float camera_fx = camera_K_.at<float>(0, 0);
	float camera_fy = camera_K_.at<float>(1, 1);
	float camera_cx = camera_K_.at<float>(0, 2);
	float camera_cy = camera_K_.at<float>(1, 2);

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;

	for (int32_t i = 0; i < matches_size; i++)
	{
		uint16_t depth = p_query_frame.point_depth_[matches[i].queryIdx];
		if (depth == 0) continue;

		query_frame_points.push_back(cv::Point3f(p_query_frame.point_3d_[matches[i].queryIdx]));
		train_frame_points.push_back(cv::Point2f(p_train_frame.key_points_[matches[i].trainIdx].pt));
	}

	if ((query_frame_points.size() == 0) || (train_frame_points.size() == 0))
	{
		return 0;
	}

	cv::Mat inliers, rotation, translation;
	cv::solvePnPRansac(query_frame_points, train_frame_points, camera_K_, camera_D_,
		rotation, translation, false, pnp_iterations_count_, pnp_error_, pnp_min_inliers_count_, inliers);

	cv::Mat rotation_3x3;
	Rodrigues(rotation, rotation_3x3);
	Eigen::Matrix3d rotation_eigen;
	cv::cv2eigen(rotation_3x3, rotation_eigen);
	p_transform = Eigen::Isometry3d::Identity();
	Eigen::AngleAxisd angle(rotation_eigen);
	p_transform = angle;
	p_transform(0, 3) = translation.at<double>(0, 0);
	p_transform(1, 3) = translation.at<double>(1, 0);
	p_transform(2, 3) = translation.at<double>(2, 0);

	return inliers.rows;
}
