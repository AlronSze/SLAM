#pragma once

#include <vector>
#include <opencv2/features2d/features2d.hpp>

#include "frame.h"

class ORBMatcher
{
public:
	static std::vector<cv::DMatch> MatchTwoFrame(const Frame & p_query_frame, const Frame & p_train_frame, const float p_match_ratio);

private:
	static std::vector<cv::DMatch> DoRansacMatch(const Frame & p_query_frame, const Frame & p_train_frame, const std::vector<cv::DMatch> p_matches);
};
