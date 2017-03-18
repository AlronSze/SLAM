#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QVTKWidget.h>

#include <opencv2/core/core.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <OpenNI.h>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"

#include "../inc/map.h"
#include "../inc/parameter.h"

class System
{
public:
	System(openni::VideoStream *p_color_stream, openni::VideoStream *p_depth_stream);
	~System();

	void SetQTDrawWidget(QLabel *p_label_color, QLabel *p_label_depth, QVTKWidget *p_vtk_widget, pcl::visualization::PCLVisualizer::Ptr p_pcl_viewer);
	void SetQTStatusWidget(QLineEdit *p_value_track_status, QLineEdit *p_value_frame_count, QLineEdit *p_value_keyframe_count, QLineEdit *p_value_loop_count);
	void SetParameter(Parameter *p_parameter);
	void SetBoWVocabulary(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary);
	void Run();

private:
	void MirrorImage(const cv::Mat &p_source, cv::Mat &p_destination);

public:
	QLabel *label_color_;
	QLabel *label_depth_;

	QLineEdit *value_track_status_;
	QLineEdit *value_frame_count_;
	QLineEdit *value_keyframe_count_;
	QLineEdit *value_loop_count_;

	QVTKWidget *vtk_widget_;
	pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;

	bool is_running_;

	Map *map_;
	Parameter *parameter_;

	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *bow_vocabulary_;

private:
	openni::VideoStream *color_stream_;
	openni::VideoStream *depth_stream_;
};

inline void System::SetQTDrawWidget(QLabel *p_label_color, QLabel *p_label_depth, QVTKWidget *p_vtk_widget, pcl::visualization::PCLVisualizer::Ptr p_pcl_viewer)
{
	label_color_ = p_label_color;
	label_depth_ = p_label_depth;
	vtk_widget_ = p_vtk_widget;
	pcl_viewer_ = p_pcl_viewer;
}

inline void System::SetQTStatusWidget(QLineEdit *p_value_track_status, QLineEdit *p_value_frame_count, QLineEdit *p_value_keyframe_count, QLineEdit *p_value_loop_count)
{
	value_track_status_ = p_value_track_status;
	value_frame_count_ = p_value_frame_count;
	value_keyframe_count_ = p_value_keyframe_count;
	value_loop_count_ = p_value_loop_count;
}

inline void System::SetParameter(Parameter *p_parameter)
{
	parameter_ = p_parameter;
}

inline void System::SetBoWVocabulary(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *p_bow_vocabulary)
{
	bow_vocabulary_ = p_bow_vocabulary;
}
