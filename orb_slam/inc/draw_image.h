#pragma once

#include <string>

#include "frame.h"

class DrawImage {
public:
	DrawImage(const std::string p_rgb_name, const std::string p_depth_name, const bool p_show_key_point = false);
	void toDrawFrame(const Frame &p_frame, const int32_t p_wait_time);

private:
	std::string rgb_name_;
	std::string depth_name_;
	std::string key_point_name_;
	bool show_key_point_;
};
