#include "../inc/optimizer.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

void Optimizer::PnPSolver(const std::vector<cv::Point3f>& p_object_points, const std::vector<cv::Point2f>& p_image_points, const cv::Mat p_camera_k, std::vector<int32_t>& inliersIndex, Eigen::Isometry3d & p_transform)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType * linear_solver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg * algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer.setAlgorithm(algorithm);

	Eigen::Isometry3d transform_init = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d rotation_init = transform_init.rotation();
	Eigen::Vector3d translation_init(transform_init(0, 3), transform_init(1, 3), transform_init(2, 3));
	g2o::VertexSE3Expmap * vertex = new g2o::VertexSE3Expmap();
	vertex->setEstimate(g2o::SE3Quat(rotation_init, translation_init));
	vertex->setId(0);
	vertex->setFixed(false);
	optimizer.addVertex(vertex);

	std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;

	const int32_t image_size = (int32_t)p_image_points.size();
	const int32_t object_size = (int32_t)p_object_points.size();
	const double camera_fx = (double)p_camera_k.at<float>(0, 0);
	const double camera_fy = (double)p_camera_k.at<float>(1, 1);
	const double camera_cx = (double)p_camera_k.at<float>(0, 2);
	const double camera_cy = (double)p_camera_k.at<float>(1, 2);
	const float delta = sqrt(5.991);

	std::vector<bool> inliers(image_size, true);
	int inliers_count = object_size;

	for (int32_t i = 0; i < object_size; i++)
	{
		g2o::EdgeSE3ProjectXYZOnlyPose * edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
		edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		edge->setMeasurement(Eigen::Vector2d(p_image_points[i].x, p_image_points[i].y));
		edge->fx = (double)p_camera_k.at<float>(0, 0);
		edge->fy = (double)p_camera_k.at<float>(1, 1);
		edge->cx = (double)p_camera_k.at<float>(0, 2);
		edge->cy = (double)p_camera_k.at<float>(1, 2);
		edge->Xw = Eigen::Vector3d(p_object_points[i].x, p_object_points[i].y, p_object_points[i].z);
		edge->setInformation(Eigen::Matrix2d::Identity() * 1);
		g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber();
		edge->setRobustKernel(robust_kernel);
		robust_kernel->setDelta(delta);
		optimizer.addEdge(edge);
		edge->setId(i);
		edges.push_back(edge);
	}

	int32_t edges_size = (int32_t)edges.size();
	for (int32_t it = 0; it < 4; it++)
	{
		Eigen::Matrix3d rotation = p_transform.rotation();
		Eigen::Vector3d translation(p_transform(0, 3), p_transform(1, 3), p_transform(2, 3));
		vertex->setEstimate(g2o::SE3Quat(rotation, translation));
		optimizer.initializeOptimization();
		optimizer.optimize(10);

		for (int32_t i = 0; i < edges_size; i++)
		{
			g2o::EdgeSE3ProjectXYZOnlyPose* edge_pose = edges[i];
			edge_pose->computeError();

			if (edge_pose->chi2() > 5.991)
			{
				inliers[i] = false;
				edge_pose->setLevel(1);
				inliers_count--;
			}
			else
			{
				inliers[i] = true;
				edge_pose->setLevel(0);
			}

			if (it == 2)
			{
				edge_pose->setRobustKernel(nullptr);
			}
		}

		if (inliers_count < 5)
		{
			break;
		}
	}

	int32_t inliers_size = (int32_t)inliers.size();
	for (int32_t i = 0; i < inliers_size; i++)
	{
		if (inliers[i])
		{
			inliersIndex.push_back(i);
		}
	}

	g2o::VertexSE3Expmap* vertex_recover = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
	g2o::SE3Quat se3_recover = vertex_recover->estimate();

	p_transform = Eigen::Isometry3d(se3_recover);
}
