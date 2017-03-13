#include "mainwindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;
	w.show();
	return a.exec();
}

//#include "inc/parameter.h"
//#include "inc/frame.h"
//#include "inc/map.h"
//#include "inc/tracking.h"
//#include "inc/loop_closing.h"
//
//#include <iostream>
//#include <thread>
//#include <opencv2/highgui/highgui.hpp>
//
//#define DEBUG_DRAW
//#ifdef DEBUG_DRAW
//#include "../inc/draw_image.h"
//#endif // DEBUG_DRAW

//int main()
//{
//	std::cout << std::endl << "Initializing SLAM..." << std::endl;
//	Parameter parameter("parameter.yml");
//	Map * map = new Map(parameter);
//	std::thread * map_thread = new std::thread(&Map::Run, map);
//	LoopClosing loop_closing(parameter, map);
//	Tracking tracking(parameter, &loop_closing);
//	std::cout << std::endl << "SLAM initialized! Start to track." << std::endl << std::endl;
//
//#ifdef DEBUG_DRAW
//	DrawImage draw_image("RGB Image", "", true);
//#endif // DEBUG_DRAW
//
//	int32_t image_number = parameter.kImageNumber_;
//
//	for (int32_t i = 1; i <= image_number; ++i)
//	{
//		std::cout << "Load image: " << i << ".png" << std::endl;
//		Frame *frame = new Frame(i, parameter);
//
//#ifdef DEBUG_DRAW
//		draw_image.toDrawFrame(*frame, 1);
//#endif // DEBUG_DRAW
//
//		tracking.GetFrame(frame);
//		delete frame;
//		std::cout << std::endl;
//	}
//
//	loop_closing.OptimizeLast();
//
//	std::cout << "Press any key to shut down SLAM." << std::endl;
//	getchar();
//
//	return 0;
//}
