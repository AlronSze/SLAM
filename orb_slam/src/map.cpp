#include "../inc/map.h"

#include <windows.h>

#include "../inc/map_point.h"

Map::Map(const Parameter & p_parameter) : can_draw_(true), draw_world_points_(false)
{
	camera_fx_ = p_parameter.kCameraParameters_.fx_;
	camera_fy_ = p_parameter.kCameraParameters_.fy_;
	camera_cx_ = p_parameter.kCameraParameters_.cx_;
	camera_cy_ = p_parameter.kCameraParameters_.cy_;
	camera_scale_ = p_parameter.kCameraParameters_.scale_;
}

void Map::GetKeyFrames(const std::vector<Frame> & p_frame, const bool p_draw_flag)
{
	key_frames_ = p_frame;
	draw_world_points_ = p_draw_flag;
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
			const int32_t key_frames_size = (int32_t)key_frames_.size();

			if (!draw_world_points_)
			{
				#pragma omp parallel for
				for (int32_t i = 0; i < key_frames_size; ++i)
				{
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloud(key_frames_[i]);
					// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloudOfWhole(key_frames_[i]);

#					pragma omp critical (section)
					{
						*global_cloud += *new_cloud;
					}
				}
			}
			else
			{
				#pragma omp parallel for
				for (int32_t i = 0; i < key_frames_size; ++i)
				{
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloudOfWorldPoints(key_frames_[i]);

					#pragma omp critical (section)
					{
						*global_cloud += *new_cloud;
					}
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

	const int32_t frame_id = p_frame.id_;

	for (size_t i = 0, for_size = p_frame.map_points_.size(); i < for_size; ++i)
	{
		MapPoint * map_point = p_frame.map_points_[i];

		if (map_point->is_bad_ || (map_point->best_id_ != frame_id))
		{
			continue;
		}

		pcl::PointXYZRGBA point_xyzrgb;

		cv::Point3f point_3f = map_point->point_3d_;
		point_xyzrgb.x = point_3f.x;
		point_xyzrgb.y = point_3f.y;
		point_xyzrgb.z = point_3f.z;
		point_xyzrgb.r = map_point->rgb_r_;
		point_xyzrgb.g = map_point->rgb_g_;
		point_xyzrgb.b = map_point->rgb_b_;

		temp->points.push_back(point_xyzrgb);
	}

	Eigen::Isometry3d transform = p_frame.GetTransform();
	pcl::transformPointCloud(*temp, *result, transform.matrix());
	result->is_dense = false;

	return result;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::GetPointCloudOfWorldPoints(const Frame & p_frame)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());

	const int32_t frame_id = p_frame.id_;

	for (size_t i = 0, for_size = p_frame.map_points_.size(); i < for_size; ++i)
	{
		MapPoint * map_point = p_frame.map_points_[i];

		if (map_point->is_bad_ || (map_point->best_id_ != frame_id))
		{
			continue;
		}

		pcl::PointXYZRGBA point_xyzrgb;

		cv::Point3f point_3f = map_point->point_world_;
		point_xyzrgb.x = point_3f.x;
		point_xyzrgb.y = point_3f.y;
		point_xyzrgb.z = point_3f.z;
		point_xyzrgb.r = map_point->rgb_r_;
		point_xyzrgb.g = map_point->rgb_g_;
		point_xyzrgb.b = map_point->rgb_b_;

		result->points.push_back(point_xyzrgb);
	}

	result->is_dense = false;

	return result;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::GetPointCloudOfWhole(const Frame & p_frame)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(p_frame.point_cloud_);

	Eigen::Isometry3d transform = p_frame.GetTransform();
	pcl::transformPointCloud(*temp, *result, transform.matrix());
	result->is_dense = false;

	return result;
}
