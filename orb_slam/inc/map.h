#pragma once

#include "frame.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class Map {
public:
	Map();
	void GetKeyFrames(const std::vector<Frame> & p_frame);

private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloud(const Frame & p_frame);
	void ShowMap();

private:
	std::vector<Frame> key_frames_;
	pcl::visualization::CloudViewer * viewer_;
};
