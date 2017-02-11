#pragma once

#include <vector>
#include <opencv2/core.hpp>

class MapPoint
{
public:
	MapPoint();

public:
	cv::Point2f point_2d_;
	cv::Point3f point_3d_;
	
	int32_t observation_count_;
	std::vector<int32_t> observation_id_;
	int32_t best_id_;

	bool is_bad_;
};
