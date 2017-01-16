#pragma once

#include <string>

class Parameter{
public:
	Parameter();
	~Parameter() {}

public:
	// image number
	int32_t image_number_;

	// camera parameters
	struct CameraParameters {
		float fx_;
		float fy_;
		float cx_;
		float cy_;
		float scale_;
		float d0_;
		float d1_;
		float d2_;
		float d3_;
		float d4_;
	} kCameraParameters_;

	// orb parameters
	int kORBFeaturesMax_;
	float kORBScale_;
	int kORBLevels_;
	int kORBThresholdInit_;
	int kORBThresholdMin_;

	// dataset directory
	std::string kDatasetDir_;

	// match ratio
	float kMatchRatio_;

	// point cloud filter parameters
	float kFilterLeafSize_;
	float kFilterDepthMax_;
};
