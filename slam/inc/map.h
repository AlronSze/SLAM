#pragma once

#include <QVTKWidget.h>

#include <mutex>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "frame.h"

class Map
{
public:
	Map();
	void GetKeyFrames(const std::vector<Frame> &p_frame, const bool p_last_flag = false);
	void Run();

private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetPointCloud(const Frame &p_frame);

public:
	bool can_draw_;
	bool vtk_flag_;
	bool last_flag_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud_;

private:
	std::vector<Frame> key_frames_;
	std::mutex mutex_;
};
