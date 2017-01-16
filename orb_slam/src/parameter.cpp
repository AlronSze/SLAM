#include "parameter.h"

#include <opencv2/core/core.hpp>

Parameter::Parameter()
{
	cv::FileStorage file_storage("parameter.yml", cv::FileStorage::READ);

	// camera parameters
	file_storage["camera"]["fx"] >> kCameraParameters_.fx_;
	file_storage["camera"]["fy"] >> kCameraParameters_.fy_;
	file_storage["camera"]["cx"] >> kCameraParameters_.cx_;
	file_storage["camera"]["cy"] >> kCameraParameters_.cy_;
	file_storage["camera"]["scale"] >> kCameraParameters_.scale_;
	file_storage["camera"]["d0"] >> kCameraParameters_.d0_;
	file_storage["camera"]["d1"] >> kCameraParameters_.d1_;
	file_storage["camera"]["d2"] >> kCameraParameters_.d2_;
	file_storage["camera"]["d3"] >> kCameraParameters_.d3_;
	file_storage["camera"]["d4"] >> kCameraParameters_.d4_;

	// orb parameters
	file_storage["orb"]["features_max"] >> kORBFeaturesMax_;
	file_storage["orb"]["scale"] >> kORBScale_;
	file_storage["orb"]["levels"] >> kORBLevels_;
	file_storage["orb"]["threshold_init"] >> kORBThresholdInit_;
	file_storage["orb"]["threshold_min"] >> kORBThresholdMin_;

	// dataset directory
	file_storage["dataset_dir"] >> kDatasetDir_;

	// match ratio
	file_storage["match_ratio"] >> kMatchRatio_;

	// point cloud filter parameters
	file_storage["filter"]["leaf_size"] >> kFilterLeafSize_;
	file_storage["filter"]["distance_max"] >> kFilterDistanceMax_;

	file_storage.release();
}
