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

	// match parameters
	float kMatchTrackRatio_;
	float kMatchRelocalizeRatio_;
	float kMatchLocalLoopRatio_;
	float kMatchGlobalLoopRatio_;

	// keyframe parameters
	double kKeyframeNormMin_;
	double kKeyframeNormMax_;

	// point cloud filter parameters
	int kFilterInterval_;
	float kFilterDepthMax_;

	// pnp parameters
	int KPNPTrackThreshold_;
	int KPNPRelocalizeThreshold_;
	int KPNPLocalLoopThreshold_;
	int KPNPGlobalLoopThreshold_;

	// dbow2 parameters
	double kDBoW2TrackScore_;
	double kDBoW2LoopScore_;

	// g2o parameters
	double kG2OChi2Threshold_;
};
