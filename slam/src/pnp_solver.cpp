#include "../inc/pnp_solver.h"

#include <iostream>
#include <map>

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../inc/orb_matcher.h"

PnPSolver::PnPSolver(const int32_t p_inliers_threshold, const float p_match_ratio, const cv::Mat &p_camera_k) :
	inliers_threshold_(p_inliers_threshold), match_ratio_(p_match_ratio), camera_k_(p_camera_k.clone()),
	transform_(Eigen::Isometry3d::Identity())
{
}

bool PnPSolver::SolvePnP(const Frame &p_query_frame, const Frame &p_train_frame, const int32_t p_pnp_method)
{
	std::vector<cv::DMatch> matches = ORBMatcher::MatchTwoFrame(p_query_frame, p_train_frame, match_ratio_);
	int32_t matches_size = (int32_t)matches.size();

	if (matches_size == 0)
	{
		return false;
	}

	std::vector<cv::Point3f> query_frame_points;
	std::vector<cv::Point2f> train_frame_points;
	query_frame_points.reserve(matches_size);
	train_frame_points.reserve(matches_size);

	for (int32_t i = 0; i < matches_size; ++i)
	{
		const int32_t query_index = matches[i].queryIdx;
		const int32_t train_index = matches[i].trainIdx;

		if (p_query_frame.GetDepth(query_index) == 0)
		{
			continue;
		}

		query_frame_points.push_back(cv::Point3f(p_query_frame.GetPoint3D(query_index)));
		train_frame_points.push_back(cv::Point2f(p_train_frame.GetPoint2D(train_index)));
	}

	if (query_frame_points.empty())
	{
		return false;
	}

	int32_t inliers_number;

	if (p_pnp_method == USING_G2O)
	{
		inliers_number = SolvePnPG2O(query_frame_points, train_frame_points);

		if (inliers_number < inliers_threshold_)
		{
			return false;
		}
	}
	else if (p_pnp_method == USING_EPNP)
	{
		cv::Mat pnp_rotation = cv::Mat::zeros(3, 1, CV_32FC1);
		cv::Mat pnp_translation = cv::Mat::zeros(3, 1, CV_32FC1);
		cv::Mat inliers_mask;

		cv::solvePnPRansac(query_frame_points, train_frame_points, camera_k_, cv::Mat(), pnp_rotation, pnp_translation, false, 400, 5.991, inliers_threshold_, inliers_mask, cv::EPNP);

		if ((inliers_number = inliers_mask.rows) < inliers_threshold_)
		{
			return false;
		}

		cv::Mat rotation_rodrigues;
		cv::Rodrigues(pnp_rotation, rotation_rodrigues);
		Eigen::Matrix3d rotation;
		cv::cv2eigen(rotation_rodrigues, rotation);
		Eigen::AngleAxisd angle_axis(rotation);

		transform_ = angle_axis;
		transform_(0, 3) = pnp_translation.at<double>(0, 0);
		transform_(1, 3) = pnp_translation.at<double>(1, 0);
		transform_(2, 3) = pnp_translation.at<double>(2, 0);
	}
	else
	{
		return false;
	}

	return true;
}

int32_t PnPSolver::SolvePnPG2O(const std::vector<cv::Point3f> &p_object_points, const std::vector<cv::Point2f> &p_image_points)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linear_solver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer.setAlgorithm(algorithm);

	Eigen::Isometry3d transform_init = transform_;
	Eigen::Matrix3d rotation_init = transform_init.rotation();
	Eigen::Vector3d translation_init(transform_init(0, 3), transform_init(1, 3), transform_init(2, 3));

	g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
	vertex->setEstimate(g2o::SE3Quat(rotation_init, translation_init));
	vertex->setId(0);
	vertex->setFixed(false);
	optimizer.addVertex(vertex);

	const int32_t object_size = (int32_t)p_object_points.size();
	const int32_t image_size = (int32_t)p_image_points.size();
	const double camera_fx = (double)camera_k_.at<float>(0, 0);
	const double camera_fy = (double)camera_k_.at<float>(1, 1);
	const double camera_cx = (double)camera_k_.at<float>(0, 2);
	const double camera_cy = (double)camera_k_.at<float>(1, 2);
	const float delta = sqrt(5.991);

	std::vector<g2o::EdgeSE3ProjectXYZOnlyPose *> edges(object_size, NULL);

	for (int32_t i = 0; i < object_size; ++i)
	{
		g2o::EdgeSE3ProjectXYZOnlyPose *edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
		edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		edge->setMeasurement(Eigen::Vector2d((double)p_image_points[i].x, (double)p_image_points[i].y));
		edge->fx = camera_fx;
		edge->fy = camera_fy;
		edge->cx = camera_cx;
		edge->cy = camera_cy;
		edge->Xw = Eigen::Vector3d((double)p_object_points[i].x, (double)p_object_points[i].y, (double)p_object_points[i].z);
		edge->setInformation(Eigen::Matrix2d::Identity());
		g2o::RobustKernelHuber *robust_kernel = new g2o::RobustKernelHuber();
		edge->setRobustKernel(robust_kernel);
		robust_kernel->setDelta(delta);
		optimizer.addEdge(edge);
		edge->setId(i);
		edges[i] = edge;
	}

	int32_t outliers;
	std::vector<int8_t> inliers_mask(object_size, 1);

	for (int32_t count = 0; count < 4; ++count)
	{
		Eigen::Matrix3d rotation = transform_.rotation();
		Eigen::Vector3d translation(transform_(0, 3), transform_(1, 3), transform_(2, 3));
		vertex->setEstimate(g2o::SE3Quat(rotation, translation));
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

		outliers = 0;

		#pragma omp parallel for
		for (int32_t i = 0; i < object_size; ++i)
		{
			g2o::EdgeSE3ProjectXYZOnlyPose *edge_pose = edges[i];

			if (!inliers_mask[i])
			{
				edge_pose->computeError();
			}

			if (edge_pose->chi2() > 5.991)
			{
				inliers_mask[i] = 0;
				edge_pose->setLevel(1);
				++outliers;
			}
			else
			{
				inliers_mask[i] = 1;
				edge_pose->setLevel(0);
			}

			if (count == 2)
			{
				edge_pose->setRobustKernel(nullptr);
			}
		}

		if ((image_size - outliers) < 10)
		{
			break;
		}
	}

	g2o::VertexSE3Expmap *vertex_recovery = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
	g2o::SE3Quat se3_recovery = vertex_recovery->estimate();
	transform_ = Eigen::Isometry3d(se3_recovery.to_homogeneous_matrix());

	return image_size - outliers;
}
