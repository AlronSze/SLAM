#include "../inc/draw_image.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

DrawImage::DrawImage(QLabel *p_label_color) : label_color_(p_label_color)
{
}

DrawImage::DrawImage(QLabel *p_label_color, QLabel *p_label_depth, const double p_max_depth) :
	label_color_(p_label_color), label_depth_(p_label_depth), max_depth_(p_max_depth)
{
	for (int32_t i = 0; i < 256; ++i)
	{
		gray_table_.push_back(qRgb(i, i, i));
	}
}

void DrawImage::DrawImages(const cv::Mat &p_image_color)
{
	if (p_image_color.channels() == 3)
	{
		QImage image_color = QImage((const uint8_t *)(p_image_color.data), p_image_color.cols, p_image_color.rows,
			                        p_image_color.cols * p_image_color.channels(), QImage::Format_RGB888);

		label_color_->setPixmap(QPixmap::fromImage(image_color).scaled(label_color_->rect().size()));
	}
}

void DrawImage::DrawImages(const cv::Mat &p_image_color, const cv::Mat &p_image_depth)
{
	if (p_image_color.channels() == 3)
	{
		QImage image_color = QImage((const uint8_t *)(p_image_color.data), p_image_color.cols, p_image_color.rows,
			                        p_image_color.cols * p_image_color.channels(), QImage::Format_RGB888);

		label_color_->setPixmap(QPixmap::fromImage(image_color).scaled(label_color_->rect().size()));
	}

	if (p_image_depth.channels() == 1)
	{
		cv::Mat temp;
		p_image_depth.convertTo(temp, CV_8UC1, 255.0 / max_depth_);

		QImage image_depth = QImage((const uint8_t *)(temp.data), temp.cols, temp.rows,
			                        temp.cols * temp.channels(), QImage::Format_Indexed8);
		image_depth.setColorTable(gray_table_);

		label_depth_->setPixmap(QPixmap::fromImage(image_depth).scaled(label_depth_->rect().size()));
	}
}

void DrawImage::DrawImages(const cv::Mat &p_image_color, const cv::Mat &p_image_depth, const std::vector<cv::KeyPoint> &p_key_points)
{
	if (p_image_color.channels() == 3)
	{
		cv::Mat temp;
		cv::drawKeypoints(p_image_color, p_key_points, temp);

		QImage image_color = QImage((const uint8_t *)(temp.data), temp.cols, temp.rows,
			                        temp.cols * temp.channels(), QImage::Format_RGB888);

		label_color_->setPixmap(QPixmap::fromImage(image_color).scaled(label_color_->rect().size()));
	}

	if (p_image_depth.channels() == 1)
	{
		cv::Mat temp;
		p_image_depth.convertTo(temp, CV_8UC1, 255.0 / max_depth_);

		QImage image_depth = QImage((const uint8_t *)(temp.data), temp.cols, temp.rows,
			                        temp.cols * temp.channels(), QImage::Format_Indexed8);
		image_depth.setColorTable(gray_table_);

		label_depth_->setPixmap(QPixmap::fromImage(image_depth).scaled(label_depth_->rect().size()));
	}
}
