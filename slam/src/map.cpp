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

Map::Map() : can_draw_(true), last_flag_(false), vtk_flag_(false), debug_time_(0)
{
}

void Map::GetKeyFrames(const std::vector<Frame> &p_frame, const bool p_last_flag)
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
	clock_t s, e;

	while (1)
	{
		if (can_draw_ && ((!vtk_flag_) || last_flag_))
		{
			mutex_.lock();

			s = clock();

			global_cloud_->clear();
			const int32_t key_frames_size = (int32_t)key_frames_.size();

			#pragma omp parallel for
			for (int32_t i = 0; i < key_frames_size; ++i)
			{
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud = GetPointCloud(key_frames_[i]);

				#pragma omp critical (global_cloud_section)
				{
					*global_cloud_ += *new_cloud;
				}
			}

			key_frames_.clear();
			vtk_flag_ = true;
			can_draw_ = false;

			e = clock();
			debug_time_ += e - s;

			mutex_.unlock();
		}

		if (last_flag_)
		{
			// pcl::io::savePCDFile("./pointcloud.pcd", *global_cloud_);
			break;
		}

		thread_sleep(1000);
	}
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Map::GetPointCloud(const Frame &p_frame)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(p_frame.point_cloud_);

	Eigen::Isometry3d transform = p_frame.transform_world_to_camera_;
	pcl::transformPointCloud(*temp, *result, transform.matrix());

	result->width = result->size();
	result->height = 1;
	result->is_dense = false;

	return result;
}
