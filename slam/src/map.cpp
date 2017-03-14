#include "../inc/map.h"

#ifdef _WIN32
#include <windows.h>
#define thread_sleep(x) Sleep(x)
#elif __linux__
#include <unistd.h>
#define thread_sleep(x) usleep(x)
#endif

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <vtkRenderWindow.h>

#include "../inc/map_point.h"

Map::Map() : can_draw_(true), last_flag_(false), vtk_flag_(false)
{
}

void Map::GetKeyFrames(const std::vector<Frame> & p_frame, const bool p_last_flag)
{
	last_flag_ = p_last_flag;

	if (last_flag_)
	{
		mutex_.lock();
	}
	else if (!mutex_.try_lock())
	{
		return;
	}

	key_frames_ = p_frame;
	can_draw_ = (key_frames_.size() != 0) ? true : false;

	mutex_.unlock();
}

void Map::Run()
{
	global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

	while (1)
	{
		if (can_draw_ && ((!vtk_flag_) || last_flag_))
		{
			mutex_.lock();

			global_cloud_->clear();
			const int32_t key_frames_size = (int32_t)key_frames_.size();

			#pragma omp parallel for
			for (int32_t i = 0; i < key_frames_size; ++i)
			{
				// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloud(key_frames_[i]);
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloudOfWhole(key_frames_[i]);

				#pragma omp critical (section)
				{
					*global_cloud_ += *new_cloud;
				}
			}

			//pcl::io::savePCDFile("./pointcloud.pcd", *global_cloud_);

			key_frames_.clear();
			vtk_flag_ = true;
			can_draw_ = false;

			mutex_.unlock();
		}

		if (last_flag_)
		{
			break;
		}

		thread_sleep(1000);
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

		if (map_point->is_bad_ || (map_point->best_id_ != frame_id) || (map_point->observation_count_ <= 1))
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
