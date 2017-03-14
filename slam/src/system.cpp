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

#include "../inc/frame.h"
#include "../inc/draw_image.h"
#include "../inc/tracking.h"
#include "../inc/loop_closing.h"

System::System(openni::VideoStream * p_color_stream, openni::VideoStream * p_depth_stream) :
	map_(NULL), is_running_(false), color_stream_(p_color_stream), depth_stream_(p_depth_stream)
{
}

System::~System()
{
	if (map_)
	{
		delete map_;
	}
}

void System::Run()
{
	bool track_result;
	int32_t lost_count = 0;

	DrawImage draw_image(label_color_, label_depth_, parameter_->kCameraParameters_.scale_, true);
	map_ = new Map();
	std::thread * map_thread = new std::thread(&Map::Run, map_);
	LoopClosing loop_closing(*parameter_, bow_vocabulary_, map_, value_loop_count_);
	Tracking tracking(*parameter_, &loop_closing, value_keyframe_count_);

	std::cout << "SLAM initialized! Start to track." << std::endl << std::endl;

	openni::VideoFrameRef color_frame;
	openni::VideoFrameRef depth_frame;
	color_stream_->start();
	depth_stream_->start();

	is_running_ = true;

	for (int32_t i = 1; is_running_; ++i)
	{
		color_stream_->readFrame(&color_frame);
		depth_stream_->readFrame(&depth_frame);

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

		track_result = tracking.GetFrame(frame);
		delete frame;

		if (track_result)
		{
			lost_count = 0;
			value_track_status_->setText("Tracking");
		}
		else
		{
			++lost_count;
			if (lost_count >= 20)
			{
				value_track_status_->setText("Lost");
			}
		}
		value_frame_count_->setText(QString().setNum(i));

		std::cout << std::endl;

		thread_sleep(1);
	}

	color_stream_->stop();
	depth_stream_->stop();

	loop_closing.OptimizeLast();

	map_thread->join();
	delete map_thread;
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
