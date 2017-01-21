#pragma once

#include "frame.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class Map {
public:
	Map();
	void GetKeyFrames(const std::vector<Frame> & p_frame);
	void Run();

private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloud(const Frame & p_frame);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloudForWhole(const Frame & p_frame);

public:
	bool can_draw_;

private:
	std::vector<Frame> key_frames_;
};
