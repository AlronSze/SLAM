#include "../inc/optimizer.h"

#include <iostream>
#include <map>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "../inc/map_point.h"

int32_t Optimizer::PnPSolver(const std::vector<cv::Point3f>& p_object_points, const std::vector<cv::Point2f>& p_image_points, const cv::Mat p_camera_k, std::vector<int8_t> & p_inliers_mask, Eigen::Isometry3d & p_transform)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType * linear_solver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg * algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer.setAlgorithm(algorithm);

	Eigen::Isometry3d transform_init = p_transform;
	Eigen::Matrix3d rotation_init = transform_init.rotation();
	Eigen::Vector3d translation_init(transform_init(0, 3), transform_init(1, 3), transform_init(2, 3));
	g2o::VertexSE3Expmap * vertex = new g2o::VertexSE3Expmap();
	vertex->setEstimate(g2o::SE3Quat(rotation_init, translation_init));
	vertex->setId(0);
	vertex->setFixed(false);
	optimizer.addVertex(vertex);

	const int32_t image_size = (int32_t)p_image_points.size();
	const int32_t object_size = (int32_t)p_object_points.size();
	const double camera_fx = (double)p_camera_k.at<float>(0, 0);
	const double camera_fy = (double)p_camera_k.at<float>(1, 1);
	const double camera_cx = (double)p_camera_k.at<float>(0, 2);
	const double camera_cy = (double)p_camera_k.at<float>(1, 2);
	const float delta = sqrt(5.991);

	std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;
	edges.reserve(object_size);

	for (int32_t i = 0; i < object_size; ++i)
	{
		g2o::EdgeSE3ProjectXYZOnlyPose * edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
		edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		edge->setMeasurement(Eigen::Vector2d((double)p_image_points[i].x, (double)p_image_points[i].y));
		edge->fx = camera_fx;
		edge->fy = camera_fy;
		edge->cx = camera_cx;
		edge->cy = camera_cy;
		edge->Xw = Eigen::Vector3d((double)p_object_points[i].x, (double)p_object_points[i].y, (double)p_object_points[i].z);
		edge->setInformation(Eigen::Matrix2d::Identity());
		g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber();
		edge->setRobustKernel(robust_kernel);
		robust_kernel->setDelta(delta);
		optimizer.addEdge(edge);
		edge->setId(i);
		edges.push_back(edge);
	}

	int32_t outliers;
	const int32_t edges_size = (int32_t)edges.size();

	for (int32_t count = 0; count < 4; ++count)
	{
		Eigen::Matrix3d rotation = p_transform.rotation();
		Eigen::Vector3d translation(p_transform(0, 3), p_transform(1, 3), p_transform(2, 3));
		vertex->setEstimate(g2o::SE3Quat(rotation, translation));
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

		outliers = 0;

		#pragma omp parallel for
		for (int32_t i = 0; i < edges_size; ++i)
		{
			g2o::EdgeSE3ProjectXYZOnlyPose* edge_pose = edges[i];

			if (!p_inliers_mask[i])
			{
				edge_pose->computeError();
			}

			if (edge_pose->chi2() > 5.991)
			{
				p_inliers_mask[i] = 0;
				edge_pose->setLevel(1);
				++outliers;
			}
			else
			{
				p_inliers_mask[i] = 1;
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

	g2o::VertexSE3Expmap* vertex_recover = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
	g2o::SE3Quat se3_recover = vertex_recover->estimate();
	p_transform = Eigen::Isometry3d(se3_recover.to_homogeneous_matrix());

	return image_size - outliers;
}

void Optimizer::BundleAdjustment(std::vector<Frame> & p_frames, const int32_t p_iteration_times, const int32_t p_start_index)
{
	const int32_t frames_size = (int32_t)p_frames.size();
	if ((frames_size - p_start_index) <= 1)
	{
		return;
	}

	const float camera_fx = p_frames[p_start_index].camera_fx_;
	const float camera_fy = p_frames[p_start_index].camera_fy_;
	const float camera_cx = p_frames[p_start_index].camera_cx_;
	const float camera_cy = p_frames[p_start_index].camera_cy_;
	const float huber_delta = sqrt(5.991);

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);

	std::map<int32_t, int32_t> frame_id_to_index;

	for (int32_t i = p_start_index; i < frames_size; ++i)
	{
		Eigen::Isometry3d transform = p_frames[i].GetTransform().inverse();
		Eigen::Matrix3d rotation = transform.rotation();
		Eigen::Vector3d translation(transform(0, 3), transform(1, 3), transform(2, 3));
		g2o::VertexSE3Expmap * vertex = new g2o::VertexSE3Expmap();
		vertex->setEstimate(g2o::SE3Quat(rotation, translation));
		vertex->setId(i);
		vertex->setFixed(i==p_start_index);
		optimizer.addVertex(vertex);
		frame_id_to_index.insert(std::make_pair(p_frames[i].id_, i));
	}

	std::map<int32_t, int32_t>::iterator frame_id_to_index_end = frame_id_to_index.end();
	std::vector<g2o::EdgeSE3ProjectXYZ *> edges;

	for (int32_t i = p_start_index, ba_point_count = 0; i < frames_size; ++i)
	{
		Frame & cur_frame = p_frames[i];

		Eigen::Matrix4d transform = cur_frame.GetTransform().matrix();

		const int32_t frame_id = cur_frame.id_;
		const int32_t map_points_size = (int32_t)cur_frame.key_point_number_;

		for (int32_t j = 0; j < map_points_size; ++j)
		{
			MapPoint *const cur_map_point = cur_frame.map_points_[j];

			if ((!cur_map_point->is_bad_) && (cur_map_point->best_id_ == frame_id) && (cur_map_point->observation_count_ > 1))
			{
				const int32_t point_id = frames_size + ba_point_count;

				cv::Point3d point_3d;
				cv::Point3f point_3f = cur_map_point->point_3d_;

				point_3d.x = transform(0, 0) * (double)point_3f.x + transform(0, 1) * (double)point_3f.y + transform(0, 2) * (double)point_3f.z + transform(0, 3);
				point_3d.y = transform(1, 0) * (double)point_3f.x + transform(1, 1) * (double)point_3f.y + transform(1, 2) * (double)point_3f.z + transform(1, 3);
				point_3d.z = transform(2, 0) * (double)point_3f.x + transform(2, 1) * (double)point_3f.y + transform(2, 2) * (double)point_3f.z + transform(2, 3);

				g2o::VertexSBAPointXYZ * vertex = new g2o::VertexSBAPointXYZ();
				vertex->setEstimate(Eigen::Vector3d(point_3d.x, point_3d.y, point_3d.z));
				vertex->setId(point_id);
				vertex->setMarginalized(true);
				optimizer.addVertex(vertex);
				
				for (auto mit : cur_map_point->observation_id_)
				{
					std::map<int32_t, int32_t>::iterator find_result;
					if ((find_result = frame_id_to_index.find(mit.first)) == frame_id_to_index_end)
					{
						continue;
					}

					const int32_t index = find_result->second;
					const cv::Point2f point_2f = p_frames[index].point_2d_[mit.second];

					g2o::EdgeSE3ProjectXYZ * edge = new g2o::EdgeSE3ProjectXYZ();
					edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(point_id)));
					edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(index)));
					edge->setMeasurement(Eigen::Vector2d((double)point_2f.x, (double)point_2f.y));
					edge->setInformation(Eigen::Matrix2d::Identity());
					g2o::RobustKernelHuber * robust_kernel = new g2o::RobustKernelHuber;
					edge->setRobustKernel(robust_kernel);
					robust_kernel->setDelta(huber_delta);
					edge->fx = camera_fx;
					edge->fy = camera_fy;
					edge->cx = camera_cx;
					edge->cy = camera_cy;

					optimizer.addEdge(edge);
					edges.push_back(edge);
				}

				++ba_point_count;
			}
		}
	}

	std::cout << "Optimizing..." << std::endl;
	optimizer.initializeOptimization();
	optimizer.optimize(p_iteration_times);

	int32_t outliers = 0;
	for (int32_t i = 0, for_size = (int32_t)edges.size(); i < for_size; ++i)
	{
		if (edges[i]->chi2() > 5.991)
		{
			edges[i]->setLevel(1);
			++outliers;
		}
		else
		{
			edges[i]->setLevel(0);
		}
	}
	std::cout << "Total edges: " << edges.size() << ", outliers: " << outliers << std::endl;
	std::cout << "Optimizing(2)..." << std::endl;
	optimizer.initializeOptimization(0);
	optimizer.optimize(p_iteration_times / 2);

	for (int32_t i = p_start_index, ba_point_count = 0; i < frames_size; ++i)
	{
		Frame & cur_frame = p_frames[i];

		g2o::VertexSE3Expmap * vertex_se3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));
		g2o::SE3Quat estimate_se3 = vertex_se3->estimate();
		cur_frame.SetTransform(Eigen::Isometry3d(estimate_se3.to_homogeneous_matrix()).inverse());

		const int32_t frame_id = cur_frame.id_;
		const int32_t map_points_size = (int32_t)cur_frame.key_point_number_;

		for (int32_t j = 0; j < map_points_size; ++j)
		{
			MapPoint *const cur_map_point = cur_frame.map_points_[j];

			if ((!cur_map_point->is_bad_) && (cur_map_point->best_id_ == frame_id) && (cur_map_point->observation_count_ > 1))
			{
				g2o::VertexSBAPointXYZ * vertex_sba = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(frames_size + ba_point_count));
				g2o::Vector3D estimate_xyz = vertex_sba->estimate();

				cv::Point3f point_3f((float)estimate_xyz(0), (float)estimate_xyz(1), (float)estimate_xyz(2));
				cur_map_point->point_world_ = point_3f;

				++ba_point_count;
			}
		}
	}
}
