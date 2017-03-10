#include "../inc/draw_image.h"

#include <opencv2/highgui/highgui.hpp>

DrawImage::DrawImage(const std::string p_rgb_name, const std::string p_depth_name, const bool p_show_key_point) :
	rgb_name_(p_rgb_name), depth_name_(p_depth_name), key_point_name_(p_rgb_name + " - KeyPoint"), show_key_point_(p_show_key_point)
{
	cv::namedWindow(p_rgb_name, cv::WINDOW_NORMAL);
	if (!p_depth_name.empty())
	{
		cv::namedWindow(p_depth_name, cv::WINDOW_NORMAL);
	}
	if (p_show_key_point)
	{
		cv::namedWindow(p_rgb_name + " - KeyPoint", cv::WINDOW_NORMAL);
	}
}

void DrawImage::toDrawFrame(const Frame & p_frame, const int32_t p_wait_time)
{
	cv::imshow(rgb_name_, p_frame.bgr_image_);
	if (!depth_name_.empty())
	{
		cv::imshow(depth_name_, p_frame.depth_image_);
	}
	if (show_key_point_)
	{
		cv::Mat key_point_image;
		cv::drawKeypoints(p_frame.bgr_image_, p_frame.key_points_, key_point_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		cv::imshow(key_point_name_, key_point_image);
	}
	cv::waitKey(p_wait_time);
}
