#include "../inc/map.h"

#include <windows.h>

Map::Map(const Parameter & p_parameter) : can_draw_(true)
{
	camera_fx_ = p_parameter.kCameraParameters_.fx_;
	camera_fy_ = p_parameter.kCameraParameters_.fy_;
	camera_cx_ = p_parameter.kCameraParameters_.cx_;
	camera_cy_ = p_parameter.kCameraParameters_.cy_;
	camera_scale_ = p_parameter.kCameraParameters_.scale_;
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
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	while (1)
	{
		if (!can_draw_)
		{
			int32_t key_frames_size = (int32_t)key_frames_.size();

			#pragma omp parallel for
			for (int32_t i = 0; i < key_frames_size; i++)
			{
				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloud(key_frames_[i]);
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloudForWhole(key_frames_[i]);

				#pragma omp critical (section)
				{
					*global_cloud += *new_cloud;
				}
			}
			viewer.showCloud(global_cloud);

			global_cloud->clear();
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
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(p_frame.point_cloud_);

	Eigen::Isometry3d T = p_frame.GetTransform();
	pcl::transformPointCloud(*temp, *result, T.matrix());
	result->is_dense = false;

	return result;
}
