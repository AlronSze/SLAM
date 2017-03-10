#include "../inc/parameter.h"
#include "../inc/frame.h"
#include "../inc/map.h"
#include "../inc/tracking.h"
#include "../inc/loop_closing.h"

#include <iostream>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <OpenNI.h>

#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../inc/draw_image.h"
#endif // DEBUG_DRAW

void SolveMirror(const cv::Mat & p_source, cv::Mat & p_destination)
{
	p_destination.create(p_source.rows, p_source.cols, p_source.type());

	int32_t rows = p_source.rows;
	int32_t cols = p_source.cols;
	int32_t channels = p_source.channels();

	if (channels == 1)
	{
		const cv::Vec2b * value_before;
		cv::Vec2b * value_after;

		for (int32_t i = 0; i < rows; ++i)
		{
			value_before = p_source.ptr<cv::Vec2b>(i);
			value_after = p_destination.ptr<cv::Vec2b>(i);

			for (int32_t j = 0; j < cols; ++j)
			{
				value_after[j] = value_before[cols - 1 - j];
			}
		}
	}
	else if (channels == 3)
	{
		const cv::Vec3b * value_before;
		cv::Vec3b * value_after;

		for (int32_t i = 0; i < rows; ++i)
		{
			value_before = p_source.ptr<cv::Vec3b>(i);
			value_after = p_destination.ptr<cv::Vec3b>(i);

			for (int32_t j = 0; j < cols; ++j)
			{
				value_after[j] = value_before[cols - 1 - j];
			}
		}
	}
}

int main()
{
	std::cout << "Initializing OpenNI..." << std::endl;

	openni::OpenNI::initialize();

	openni::Device any_device;
	if (any_device.open(openni::ANY_DEVICE) != openni::STATUS_OK)
	{
		std::cout << "Cannot find any device, please try again!" << std::endl;
		return 0;
	}

	openni::VideoStream color_stream, depth_stream;
	color_stream.create(any_device, openni::SENSOR_COLOR);
	depth_stream.create(any_device, openni::SENSOR_DEPTH);

	openni::VideoMode color_mode;
	color_mode.setResolution(640, 480);
	color_mode.setFps(30);
	color_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	color_stream.setVideoMode(color_mode);
	color_stream.setMirroringEnabled(false);

	openni::VideoMode depth_mode;
	depth_mode.setResolution(640, 480);
	depth_mode.setFps(30);
	depth_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	depth_stream.setVideoMode(depth_mode);
	depth_stream.setMirroringEnabled(false);

	if (any_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		any_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	std::cout << "OpenNI Initialized!" << std::endl << std::endl;


	std::cout << "Initializing SLAM..." << std::endl;

	Parameter parameter("parameter.yml");
	Map * map = new Map(parameter);
	std::thread * map_thread = new std::thread(&Map::Run, map);
	LoopClosing loop_closing(parameter, map);
	Tracking tracking(parameter, &loop_closing);

	std::cout << "SLAM initialized! Start to track." << std::endl << std::endl;

#ifdef DEBUG_DRAW
	DrawImage draw_image("RGB Image", "Depth Image", true);
#endif // DEBUG_DRAW

	color_stream.start();
	depth_stream.start();

	openni::VideoFrameRef color_frame, depth_frame;

	for (int32_t i = 1; ; ++i)
	{
		color_stream.readFrame(&color_frame);
		depth_stream.readFrame(&depth_frame);

		cv::Mat rgb_image(color_frame.getHeight(), color_frame.getWidth(), CV_8UC3, (void *)color_frame.getData());
		cv::Mat depth_image(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1, (void *)depth_frame.getData());

		cv::Mat bgr_image, bgr_image_mirror, depth_image_mirror;
		cv::cvtColor(rgb_image, bgr_image, CV_RGB2BGR);

		SolveMirror(bgr_image, bgr_image_mirror);
		SolveMirror(depth_image, depth_image_mirror);

		std::cout << "Load image, ID: " << i << std::endl;
		Frame * frame = new Frame(i, bgr_image_mirror, depth_image_mirror, parameter);

#ifdef DEBUG_DRAW
		draw_image.toDrawFrame(*frame, 1);
#endif // DEBUG_DRAW

		tracking.GetFrame(frame);
		delete frame;
		std::cout << std::endl;

		if (cv::waitKey(1) == 'q')
		{
			break;
		}
	}

	color_stream.destroy();
	depth_stream.destroy();

	any_device.close();

	openni::OpenNI::shutdown();

	loop_closing.OptimizeLast();

	std::cout << "Press any key to shut down SLAM." << std::endl;
	getchar();

	return 0;
}
