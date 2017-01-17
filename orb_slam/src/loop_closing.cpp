#include "../inc/loop_closing.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

LoopClosing::LoopClosing(const Parameter & p_parameter)
{
	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>* linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	linear_solver->setBlockOrdering(false);
	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);
}

void LoopClosing::GetKeyFrame(const Frame & p_frame)
{
	cur_frame_ = Frame(p_frame);

	if (key_frames_.size() == 0)
	{
		key_frames_.push_back(cur_frame_);
		g2o::VertexSE3* vertex = new g2o::VertexSE3();
		vertex->setId(cur_frame_.id_);
		vertex->setEstimate(cur_frame_.GetTransform().inverse());
		vertex->setFixed(true);
		optimizer.addVertex(vertex);
	}
	else
	{
		AddCurFrameToGraph();
		LoopClose();
	}
}

void LoopClosing::AddCurFrameToGraph()
{
	g2o::VertexSE3* vertex = new g2o::VertexSE3();
	vertex->setId(cur_frame_.id_);
	vertex->setEstimate(cur_frame_.GetTransform().inverse());
	vertex->setFixed(false);
	optimizer.addVertex(vertex);

	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	g2o::VertexSE3* vertex_0 = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(key_frames_.back().id_));
	g2o::VertexSE3* vertex_1 = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(cur_frame_.id_));
	edge->setVertex(0, vertex_1);
	edge->setVertex(1, vertex_0);
	edge->setMeasurementFromState();
	edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
	edge->setRobustKernel(new g2o::RobustKernelHuber());

	optimizer.addEdge(edge);
}

void LoopClosing::LoopClose()
{
}

void LoopClosing::SetBowVector(Frame & p_frame)
{
	DBoW2::FeatureVector feature_vector;
	vocabulary_.transform(p_frame.GetDescriptorVector(), p_frame.bow_vector, feature_vector, 2);
}

