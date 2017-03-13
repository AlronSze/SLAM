#include "../inc/system.h"

#ifdef _WIN32
#include <windows.h>
#define thread_sleep(x) Sleep(x)
#elif __linux__
#include <unistd.h>
#define thread_sleep(x) usleep(x)
#endif

#include <QImage>

#include <string>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <OpenNI.h>

#include "../inc/frame.h"
#include "../inc/draw_image.h"
#include "../inc/tracking.h"
#include "../inc/loop_closing.h"

void System::Run()
{
	std::cout << "Initializing OpenNI..." << std::endl;

	openni::OpenNI::initialize();

	openni::Device any_device;
	if (any_device.open(openni::ANY_DEVICE) != openni::STATUS_OK)
	{
		std::cout << "Cannot find any device, please try again!" << std::endl;
		openni::OpenNI::shutdown();
		return;
	}

	openni::VideoStream color_stream, depth_stream;
	color_stream.create(any_device, openni::SENSOR_COLOR);
	depth_stream.create(any_device, openni::SENSOR_DEPTH);

	openni::VideoMode color_mode;
	color_mode.setResolution(640, 480);
	color_mode.setFps(30);
	color_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	color_stream.setVideoMode(color_mode);
	//color_stream.setMirroringEnabled(false);

	openni::VideoMode depth_mode;
	depth_mode.setResolution(640, 480);
	depth_mode.setFps(30);
	depth_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	depth_stream.setVideoMode(depth_mode);
	//depth_stream.setMirroringEnabled(false);

	if (any_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		any_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	color_stream.start();
	depth_stream.start();

	std::cout << depth_stream.getMaxPixelValue() << std::endl;

	openni::VideoFrameRef color_frame, depth_frame;

	std::cout << "OpenNI Initialized!" << std::endl << std::endl;

	DrawImage draw_image(label_color_, label_depth_, true);
	map_ = new Map();
	std::thread * map_thread = new std::thread(&Map::Run, map_);
	LoopClosing loop_closing(*parameter_, bow_vocabulary_, map_);
	Tracking tracking(*parameter_, &loop_closing);

	std::cout << "SLAM initialized! Start to track." << std::endl << std::endl;

	is_running_ = true;

	for (int32_t i = 1; is_running_; ++i)
	{
		color_stream.readFrame(&color_frame);
		depth_stream.readFrame(&depth_frame);

		cv::Mat rgb_image(color_frame.getHeight(), color_frame.getWidth(), CV_8UC3, (void *)color_frame.getData());
		cv::Mat depth_image(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1, (void *)depth_frame.getData());

		cv::Mat rgb_image_mirror, depth_image_mirror;
		MirrorImage(rgb_image, rgb_image_mirror);
		MirrorImage(depth_image, depth_image_mirror);

		cv::Mat bgr_image_mirror;
		cv::cvtColor(rgb_image_mirror, bgr_image_mirror, CV_RGB2BGR);

		std::cout << "Load image, ID: " << i << std::endl;
		Frame * frame = new Frame(i, bgr_image_mirror, depth_image_mirror, *parameter_);

		draw_image.DrawImages(rgb_image_mirror, depth_image_mirror);

		tracking.GetFrame(frame);
		delete frame;
		std::cout << std::endl;

		thread_sleep(1);
	}

	color_stream.destroy();
	depth_stream.destroy();

	any_device.close();

	openni::OpenNI::shutdown();

	loop_closing.OptimizeLast();

	map_->is_running_ = false;
	map_thread->join();
	thread_over_ = true;
}

void System::MirrorImage(const cv::Mat & p_source, cv::Mat & p_destination)
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
