#pragma once

#include <QLabel>

#include <string>

#include <opencv2/core/core.hpp>

#include "frame.h"

class DrawImage
{
public:
	DrawImage(QLabel *p_label_color);
	DrawImage(QLabel *p_label_color, QLabel *p_label_depth, const double p_max_depth);
	void DrawImages(const cv::Mat &p_image_color);
	void DrawImages(const cv::Mat &p_image_color, const cv::Mat &p_image_depth);
	void DrawImages(const cv::Mat &p_image_color, const cv::Mat &p_image_depth, const std::vector<cv::KeyPoint> &p_key_points);

private:
	QLabel *label_color_;
	QLabel *label_depth_;
	QVector<QRgb> gray_table_;
	bool show_key_point_;
	double max_depth_;
};

