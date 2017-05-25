#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <vtkRenderWindow.h>
#include <OpenNI.h>

#include <QPushButton>
#include <QTimer>

#include <thread>

#include "3rd_part/dbow2/FORB.h"
#include "3rd_part/dbow2/TemplatedVocabulary.h"
#include "inc/parameter.h"
#include "inc/system.h"
#include "photowindow.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();

private:
	void InitializeSlots();
	void InitializeVTK();
	void InitializeOpenNI();

public slots:
	void SlotSelectYML();
	void SlotLoadYML();
	void SlotSelectVocabulary();
	void SlotLoadVocabulary();
	void SlotStartSLAM();
	void SlotStopSLAM();
	void SlotUpdateVTK();
	void SlotModifyCamera();
	void SlotRefreshDevice();
	void SlotOpenDevice();
	void SlotCloseDevice();
	void SlotEnterPhotoMode();
	void SlotDestroypPhotoWindow();

public:
	DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> *bow_vocabulary_;
	pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;
	openni::VideoStream *color_stream_;
	openni::VideoStream *depth_stream_;

	clock_t debug_time_;

private:
	Ui::MainWindowClass ui_;

	PhotoWindow *photo_window_;

	Parameter *parameter_;
	System *system_;
	std::thread *system_thread_;
	QTimer vtk_timer_;

	openni::Device rgbd_device_;
	openni::Array<openni::DeviceInfo> rgbd_device_list_;

	bool dev_flag_;
	bool yml_flag_;
	bool voc_flag_;
	bool is_photo_mode_;
};
