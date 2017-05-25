#include "../inc/parameter.h"

#include <iostream>
#include <opencv2/core/core.hpp>

bool Parameter::LoadYMLFile(const std::string p_yml_name)
{
	cv::FileStorage file_storage(p_yml_name, cv::FileStorage::READ);
	if (!file_storage.isOpened())
	{
		return false;
	}

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

	// match parameters
	file_storage["match"]["track_ratio"] >> kMatchTrackRatio_;
	file_storage["match"]["relocalize_ratio"] >> kMatchRelocalizeRatio_;
	file_storage["match"]["local_loop_ratio"] >> kMatchLocalLoopRatio_;
	file_storage["match"]["global_loop_ratio"] >> kMatchGlobalLoopRatio_;

	// keyframe parameters
	file_storage["keyframe"]["norm_min"] >> kKeyframeNormMin_;
	file_storage["keyframe"]["norm_max"] >> kKeyframeNormMax_;

	// point cloud filter parameters
	file_storage["filter"]["interval"] >> kFilterInterval_;
	file_storage["filter"]["depth_max"] >> kFilterDepthMax_;

	// pnp parameters
	file_storage["pnp"]["track_threshold"] >> KPNPTrackThreshold_;
	file_storage["pnp"]["relocalize_threshold"] >> KPNPRelocalizeThreshold_;
	file_storage["pnp"]["local_loop_threshold"] >> KPNPLocalLoopThreshold_;
	file_storage["pnp"]["global_loop_threshold"] >> KPNPGlobalLoopThreshold_;

	// dbow2 parameters
	file_storage["dbow2"]["track_score"] >> kDBoW2TrackScore_;
	file_storage["dbow2"]["loop_score"] >> kDBoW2LoopScore_;

	// g2o parameters
	file_storage["g2o"]["chi2_threshold"] >> kG2OChi2Threshold_;

	file_storage.release();

	// Print();

	return true;
}

void Parameter::Print()
{
	std::cout << std::endl << "===================== Parameters =====================" << std::endl;

	std::cout << "Camera Intrinsic fx    : " << kCameraParameters_.fx_ << std::endl;
	std::cout << "Camera Intrinsic fy    : " << kCameraParameters_.fy_ << std::endl;
	std::cout << "Camera Intrinsic cx    : " << kCameraParameters_.cx_ << std::endl;
	std::cout << "Camera Intrinsic cy    : " << kCameraParameters_.cy_ << std::endl;
	std::cout << "Camera Intrinsic d0    : " << kCameraParameters_.d0_ << std::endl;
	std::cout << "Camera Intrinsic d1    : " << kCameraParameters_.d1_ << std::endl;
	std::cout << "Camera Intrinsic d2    : " << kCameraParameters_.d2_ << std::endl;
	std::cout << "Camera Intrinsic d3    : " << kCameraParameters_.d3_ << std::endl;
	std::cout << "Camera Intrinsic d4    : " << kCameraParameters_.d4_ << std::endl;
	std::cout << "Camera Intrinsic scale : " << kCameraParameters_.scale_ << std::endl;

	std::cout << "======================================================" << std::endl << std::endl;
}
