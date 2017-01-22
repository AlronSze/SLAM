#pragma once

#include "frame.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "parameter.h"

class Map {
public:
	Map(const Parameter & p_parameter);
	void GetKeyFrames(const std::vector<Frame> & p_frame);
	void Run();

private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloud(const Frame & p_frame);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloudForWhole(const Frame & p_frame);

public:
	bool can_draw_;
	float camera_fx_;
	float camera_fy_;
	float camera_cx_;
	float camera_cy_;
	float camera_scale_;

private:
	std::vector<Frame> key_frames_;
};
