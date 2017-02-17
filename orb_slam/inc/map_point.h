#pragma once

#include <set>
#include <opencv2/core.hpp>

class MapPoint
{
public:
	MapPoint();

	void InsertObservation(const int32_t p_frame_id);
	void EraseObservation(const int32_t p_frame_id);

public:
	cv::Point2f point_2d_;
	cv::Point3f point_3d_;
	
	uint8_t rgb_r_;
	uint8_t rgb_g_;
	uint8_t rgb_b_;

	std::set<int32_t> observation_id_;

	int32_t observation_count_;
	int32_t best_id_;

	bool is_bad_;
};

inline void MapPoint::InsertObservation(const int32_t p_frame_id)
{
	if (observation_id_.insert(p_frame_id).second)
	{
		++observation_count_;
		best_id_ = (p_frame_id > best_id_) ? p_frame_id : best_id_;
	}
}

inline void MapPoint::EraseObservation(const int32_t p_frame_id)
{
	if (observation_id_.erase(p_frame_id))
	{
		--observation_count_;
		best_id_ = *(--observation_id_.end());
	}
}
