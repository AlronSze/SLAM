#include "../inc/map.h"

//#include <pcl/filters/voxel_grid.h>
#include <windows.h>

Map::Map() : can_draw_(true)
{
}

void Map::GetKeyFrames(const std::vector<Frame> & p_frame)
{
	key_frames_ = p_frame;
	if (key_frames_.size() != 0)
	{
		can_draw_ = false;
	}
}

void Map::Run()
{
	pcl::visualization::CloudViewer viewer("viewer");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>);

	while (1)
	{
		if (!can_draw_)
		{
			for (auto key_frame : key_frames_)
			{
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = GetPointCloudForWhole(key_frame);
				*global_map += *temp;
			}
			viewer.showCloud(global_map);

			global_map->clear();
			temp->clear();
			key_frames_.clear();

			can_draw_ = true;
		}

		Sleep(3000);
	}
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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::GetPointCloudForWhole(const Frame & p_frame)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());

	int32_t points_number = (int32_t)p_frame.point_3d_.size();

	for (int32_t y = 0; y < p_frame.depth_image_.rows; y += 3)
	{
		for (int32_t x = 0; x < p_frame.depth_image_.cols; x += 3)
		{
			uint16_t depth = p_frame.depth_image_.ptr<uint16_t>(y)[x];
			if ((depth == 0) || (depth > 4 * 5000))
				continue;

			pcl::PointXYZRGBA point_xyzrgb;

			cv::Point3f point_3f;
			point_3f.z = (float)depth / 5000.0f;
			point_3f.x = ((float)x - 325.1f) * point_3f.z / 520.9f;
			point_3f.y = ((float)y - 249.7f) * point_3f.z / 521.0f;

			point_xyzrgb.b = p_frame.rgb_image_.ptr<uint8_t>(y)[x * 3];
			point_xyzrgb.g = p_frame.rgb_image_.ptr<uint8_t>(y)[x * 3 + 1];
			point_xyzrgb.r = p_frame.rgb_image_.ptr<uint8_t>(y)[x * 3 + 2];

			point_xyzrgb.x = point_3f.x;
			point_xyzrgb.y = point_3f.y;
			point_xyzrgb.z = point_3f.z;

			result->points.push_back(point_xyzrgb);
		}
	}

	Eigen::Isometry3d T = p_frame.GetTransform();
	pcl::transformPointCloud(*temp, *result, T.matrix());
	result->is_dense = false;

	return result;
}
