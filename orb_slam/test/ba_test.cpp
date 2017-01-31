//#include <iostream>
//#include "../inc/parameter.h"
//#include "../inc/frame.h"
//#include "../inc/map.h"
//#include "../inc/tracking.h"
//#include "../inc/loop_closing.h"
//
//#include <iostream>
//#include <vector>
//#include <thread>
//#include <opencv2/highgui/highgui.hpp>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/sparse_optimizer.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>
//#include <g2o/types/sba/types_six_dof_expmap.h>
//
//void BundleAdjustment(const std::vector<Frame> & p_frame, std::vector<cv::Mat> & p_map_points)
//{
//	g2o::SparseOptimizer optimizer;
//	g2o::BlockSolver_6_3::LinearSolverType *linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
//	g2o::BlockSolver_6_3 * block_solver = new g2o::BlockSolver_6_3(linear_solver);
//	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
//	optimizer.setAlgorithm(algorithm);
//	optimizer.setVerbose(false);
//
//	std::vector<g2o::EdgeSE3ProjectXYZ*> edges;
//
//	int32_t frames_size = p_frame.size();
//	for (int32_t i = 0; i < frames_size; i++)
//	{
//		Eigen::Isometry3d transform_init = p_frame[i].GetTransform();
//		Eigen::Matrix3d rotation_init = transform_init.rotation();
//		Eigen::Vector3d translation_init(transform_init(0, 3), transform_init(1, 3), transform_init(2, 3));
//		g2o::VertexSE3Expmap * vertex = new g2o::VertexSE3Expmap();
//		vertex->setEstimate(g2o::SE3Quat(rotation_init, translation_init));
//		vertex->setId(i);
//		vertex->setFixed(i==0);
//		optimizer.addVertex(vertex);
//	}
//
//	const float thHuber2D = sqrt(5.991);
//
//	size_t map_points_size = p_frame[0].point_3d_.size();
//	for (size_t i = 0; i < map_points_size; i++)
//	{
//		cv::Point3f point_3f = p_frame[0].point_3d_[i];
//
//		g2o::VertexSBAPointXYZ* vertex_point = new g2o::VertexSBAPointXYZ();
//		vertex_point->setEstimate(Eigen::Vector3d(point_3f.x, point_3f.y, point_3f.z));
//		vertex_point->setId(i + 2);
//		vertex_point->setMarginalized(true);
//		optimizer.addVertex(vertex_point);
//
//
//		g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();
//		edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + 2)));
//		edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0)));
//		edge->setMeasurement(Eigen::Vector2d(p_frame[0].point_2d_[i].x, p_frame[0].point_2d_[i].y));
//		edge->setInformation(Eigen::Matrix2d::Identity() * 1);
//		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//		edge->setRobustKernel(rk);
//		rk->setDelta(thHuber2D);
//
//		edge->fx = p_frame[0].camera_fx_;
//		edge->fy = p_frame[0].camera_fy_;
//		edge->cx = p_frame[0].camera_cx_;
//		edge->cy = p_frame[0].camera_cy_;
//
//		optimizer.addEdge(edge);
//
//		edges.push_back(edge);
//	}
//
//	size_t map_points_size2 = p_frame[1].point_3d_.size();
//	for (size_t i = 0; i < map_points_size2; i++)
//	{
//		g2o::EdgeSE3ProjectXYZ* edge2 = new g2o::EdgeSE3ProjectXYZ();
//		edge2->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + 2)));
//		edge2->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1)));
//		edge2->setMeasurement(Eigen::Vector2d(p_frame[1].point_2d_[i].x, p_frame[1].point_2d_[i].y));
//		edge2->setInformation(Eigen::Matrix2d::Identity() * 1);
//		edge2->setParameterId(0, 0);
//		g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
//		edge2->setRobustKernel(rk2);
//		rk2->setDelta(thHuber2D);
//
//		edge2->fx = p_frame[0].camera_fx_;
//		edge2->fy = p_frame[0].camera_fy_;
//		edge2->cx = p_frame[0].camera_cx_;
//		edge2->cy = p_frame[0].camera_cy_;
//
//		optimizer.addEdge(edge2);
//
//		edges.push_back(edge2);
//	}
//
//	optimizer.save("BA_test_before.g2o");
//	optimizer.initializeOptimization();
//	optimizer.optimize(20);
//	optimizer.save("BA_test_after.g2o");
//
//	for (size_t i = 0; i < map_points_size; i++)
//	{
//		g2o::VertexSBAPointXYZ* vertex_point = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + 2));
//		g2o::Vector3D result = vertex_point->estimate();
//
//		cv::Mat cvMat(3, 1, CV_32F);
//		for (int i = 0; i < 3; i++)
//		{
//			cvMat.at<float>(i) = result(i);
//		}
//
//		p_map_points.push_back(cvMat);
//	}
//
//	int count = 0;
//
//	for (size_t i = 0; i < edges.size(); i++)
//	{
//		edges[i]->computeError();
//		std::cout << "error = " << edges[i]->chi2() << std::endl;
//		if (edges[i]->chi2() < 1)
//		{
//			count++;
//		}
//	}
//
//	std::cout << "inliers: " << count << std::endl;
//}
//
//int main()
//{
//	std::cout << std::endl << "Initializing SLAM..." << std::endl;
//	Parameter parameter("parameter.yml");
//
//	std::vector<Frame> frames;
//	std::vector<cv::Mat> map_point;
//
//	for (int32_t i = 1; i <= 2; i++)
//	{
//		std::cout << "Load image: " << i << ".png" << std::endl;
//		Frame frame(i, parameter);
//		frames.push_back(frame);
//		std::cout << std::endl;
//	}
//
//	std::cout << std::endl << "Start Bundle Adjustment..." << std::endl;
//	BundleAdjustment(frames, map_point);
//
//	return 0;
//}
