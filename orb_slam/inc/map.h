#pragma once

#include "frame.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "parameter.h"

class Map
{
public:
	Map(const Parameter & p_parameter);
	void GetKeyFrames(const std::vector<Frame> & p_frame, const bool p_draw_flag);
	void Run();

private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloud(const Frame & p_frame);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloudOfWorldPoints(const Frame & p_frame);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloudOfWhole(const Frame & p_frame);

public:
	bool can_draw_;
	bool draw_world_points_;

private:
	std::vector<Frame> key_frames_;
};
