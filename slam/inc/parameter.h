#pragma once

#include <string>

class Parameter
{
public:
	Parameter() {}
	bool LoadYMLFile(const std::string p_yml_name);

private:
	void Print();

public:
	// image number
	int32_t kImageNumber_;

	// camera parameters
	struct CameraParameters
	{
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
	float kORBMatchRatio_;

	// dataset directory
	std::string kDatasetDir_;

	// point cloud filter parameters
	int kFilterInterval_;
	float kFilterDepthMax_;

	// pnp parameters
	int KPNPInliersThreshold_;

	// vocabulary directory
	std::string kVocabularyDir_;

	// dbow2 loop parameters
	double kDBoW2ScoreMin_;
	int kDBoW2IntervalMin_;

	// g2o parameters
	double kG2OChi2Threshold_;
};
