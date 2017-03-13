#pragma once

#include <QLabel>
#include <QVTKWidget.h>

#include "../3rd_part/dbow2/FORB.h"
#include "../3rd_part/dbow2/TemplatedVocabulary.h"
#include "../inc/map.h"
#include "../inc/parameter.h"

#include <opencv2/core/core.hpp>
#include <pcl/visualization/cloud_viewer.h>

class System
{
public:
	System() : map_(NULL), thread_over_(false), is_running_(false) {}
	~System() {}

	void SetQTWidget(QLabel * p_label_color, QLabel * p_label_depth, QVTKWidget * p_vtk_widget, pcl::visualization::PCLVisualizer::Ptr p_pcl_viewer);
	void SetParameter(Parameter * p_parameter);
	void SetBoWVocabulary(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> * p_bow_vocabulary);
	void Run();

private:
	void MirrorImage(const cv::Mat & p_source, cv::Mat & p_destination);

public:
	QLabel * label_color_;
	QLabel * label_depth_;
	QVTKWidget * vtk_widget_;
	pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;

	bool is_running_;
	bool thread_over_;

	Parameter * parameter_;
	Map * map_;
	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> * bow_vocabulary_;
};

inline void System::SetQTWidget(QLabel * p_label_color, QLabel * p_label_depth, QVTKWidget * p_vtk_widget, pcl::visualization::PCLVisualizer::Ptr p_pcl_viewer)
{
	label_color_ = p_label_color;
	label_depth_ = p_label_depth;
	vtk_widget_ = p_vtk_widget;
	pcl_viewer_ = p_pcl_viewer;
}

inline void System::SetParameter(Parameter * p_parameter)
{
	parameter_ = p_parameter;
}

inline void System::SetBoWVocabulary(DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>* p_bow_vocabulary)
{
	bow_vocabulary_ = p_bow_vocabulary;
}
