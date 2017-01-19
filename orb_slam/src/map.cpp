#include "../inc/map.h"

//#include <pcl/filters/voxel_grid.h>

Map::Map()
{
	viewer_ = new pcl::visualization::CloudViewer("viewer");
}

void Map::GetKeyFrames(const std::vector<Frame> & p_frame)
{
	key_frames_ = p_frame;
	ShowMap();
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::GetPointCloud(const Frame & p_frame)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());

	int32_t points_number = (int32_t)p_frame.point_3d_.size();
	for (int32_t i = 0; i < points_number; i++)
	{
		if (p_frame.point_depth_[i] == 0)
		{
			continue;
		}

		pcl::PointXYZRGBA point_xyzrgb;

		cv::Point3f point_3f = p_frame.point_3d_[i];
		point_xyzrgb.x = point_3f.x;
		point_xyzrgb.y = point_3f.y;
		point_xyzrgb.z = point_3f.z;

		FrameRGB rgb = p_frame.point_rgb_[i];
		point_xyzrgb.r = rgb.r_;
		point_xyzrgb.g = rgb.g_;
		point_xyzrgb.b = rgb.b_;

		temp->points.push_back(point_xyzrgb);
	}

	Eigen::Isometry3d T = p_frame.GetTransform();
	pcl::transformPointCloud(*temp, *result, T.matrix());
	result->is_dense = false;

	return result;
}

void Map::ShowMap()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (auto key_frame : key_frames_)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = GetPointCloud(key_frame);
		*global_map += *cloud;
	}

	viewer_->showCloud(global_map);
}
