#include "../inc/orb_matcher.h"

#include <opencv2/legacy/legacy.hpp>

std::vector<cv::DMatch> ORBMatcher::MatchTwoFrame(const Frame & p_query_frame, const Frame & p_train_frame, const float p_match_ratio)
{
	cv::BruteForceMatcher<cv::HammingLUT> matcher;
	std::vector<std::vector<cv::DMatch> > matches_knn;
	std::vector<cv::DMatch> matches;

	matcher.knnMatch(p_query_frame.descriptors_, p_train_frame.descriptors_, matches_knn, 2);

	const size_t knn_size = matches_knn.size();
	matches.reserve(knn_size);

	for (size_t i = 0; i < knn_size; ++i)
	{
		if (matches_knn[i][0].distance < (p_match_ratio * matches_knn[i][1].distance))
		{
			matches.push_back(matches_knn[i][0]);
		}
	}

	// return matches;
	return DoRansacMatch(p_query_frame, p_train_frame, matches);
}

std::vector<cv::DMatch> ORBMatcher::DoRansacMatch(const Frame & p_query_frame, const Frame & p_train_frame, const std::vector<cv::DMatch> p_matches)
{
	const size_t matches_size = p_matches.size();
	if (matches_size == 0)
	{
		return p_matches;
	}

	std::vector<cv::DMatch> ransac_matches;
	std::vector<cv::Point2f> query_points(p_matches.size());
	std::vector<cv::Point2f> train_points(p_matches.size());

	for (size_t i = 0; i < matches_size; ++i)
	{
		query_points[i] = p_query_frame.point_2d_[p_matches[i].queryIdx];
		train_points[i] = p_train_frame.point_2d_[p_matches[i].trainIdx];
	}

	std::vector<uint8_t> inliers_mask(matches_size);
	cv::findFundamentalMat(query_points, train_points, inliers_mask);

	for (size_t i = 0, for_size = inliers_mask.size(); i < for_size; ++i)
	{
		if (inliers_mask[i])
		{
			ransac_matches.push_back(p_matches[i]);
		}
	}

	return ransac_matches;
}
