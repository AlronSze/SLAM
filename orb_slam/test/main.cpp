#include "../inc/parameter.h"
#include "../inc/frame.h"
#include "../inc/map.h"
#include "../inc/tracking.h"
#include "../inc/local_mapping.h"
#include "../inc/loop_closing.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

// #define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../inc/draw_image.h"
#endif // DEBUG_DRAW

int main()
{
	std::cout << std::endl << "Initializing SLAM..." << std::endl;
	Parameter parameter("parameter.yml");
	Map map;
	// LocalMapping local_mapping;
	LoopClosing loop_closing(parameter, &map);
	Tracking tracking(parameter, &loop_closing);
	std::cout << std::endl << "SLAM initialized! Start to track." << std::endl << std::endl;

#ifdef DEBUG_DRAW
	DrawImage draw_image("RGB Image", "", false);
#endif // DEBUG_DRAW

	int32_t image_number = parameter.kImageNumber_;

	for (int32_t i = 1; i <= image_number; i++)
	{
		std::cout << "Load image: " << i << ".png" << std::endl;
		Frame *frame = new Frame(i, parameter);

#ifdef DEBUG_DRAW
		draw_image.toDrawFrame(*frame, 1);
#endif // DEBUG_DRAW
		frame->ReleaseImage();
		
		tracking.GetFrame(frame);
		delete frame;
		std::cout << std::endl;
	}

	loop_closing.SaveG2OFile("result_before.g2o");
	loop_closing.GlobalOptimize();
	loop_closing.SaveG2OFile("result_after.g2o");

	getchar();
	getchar();

    return 0;
}

// ==================== Test for G2O Method ====================
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>
//#include <g2o/types/slam3d/types_slam3d.h>
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/legacy/legacy.hpp>
//
//int main()
//{
//	g2o::SparseOptimizer optimizer;
//	g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>* linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
//	linear_solver->setBlockOrdering(false);
//	g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
//	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
//	optimizer.setAlgorithm(algorithm);
//	optimizer.setVerbose(false);
//
//	optimizer.load("test.g2o");
//	optimizer.initializeOptimization();
//	optimizer.optimize(50);
//	optimizer.save("test_after.g2o");
//
//	return 0;
//}
// =============================================================