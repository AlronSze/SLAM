#include "mainwindow.h"

#ifdef _WIN32
#include <windows.h>
#define thread_sleep(x) Sleep(x)
#elif __linux__
#include <unistd.h>
#define thread_sleep(x) usleep(x)
#endif

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

#include <string>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent), parameter_(NULL), bow_vocabulary_(NULL)
{
	ui.setupUi(this);

	pcl_viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	vtkSmartPointer<vtkRenderWindow> renderWindow = pcl_viewer_->getRenderWindow();
	ui.qvtk_widget_->SetRenderWindow(renderWindow);
	pcl_viewer_->setShowFPS(false);
	pcl_viewer_->addCoordinateSystem(0.1);
	pcl_viewer_->setCameraPosition(0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
	ui.qvtk_widget_->update();

	bow_vocabulary_ = new DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>();

	connect(ui.button_select_yml_, SIGNAL(clicked()), this, SLOT(SlotSelectYML()));
	connect(ui.button_apply_yml_, SIGNAL(clicked()), this, SLOT(SlotApplyYML()));
	connect(ui.button_select_vocabulary_, SIGNAL(clicked()), this, SLOT(SlotSelectVocabulary()));
	connect(ui.button_load_vocabulary_, SIGNAL(clicked()), this, SLOT(SlotLoadVocabulary()));
	connect(ui.button_start_, SIGNAL(clicked()), this, SLOT(SlotStartSLAM()));
	connect(ui.button_stop_, SIGNAL(clicked()), this, SLOT(SlotStopSLAM()));
	connect(ui.button_modify_camera_, SIGNAL(clicked()), this, SLOT(SlotModifyCamera()));
	
	vtk_timer_ = new QTimer;
	connect(vtk_timer_, SIGNAL(timeout()), this, SLOT(SlotUpdateVTK()));
}

void MainWindow::SlotSelectYML()
{
	QString file_name = QFileDialog::getOpenFileName(this, NULL, NULL, "*.yml");
	ui.path_yml_->setText(file_name);
	ui.button_apply_yml_->setEnabled(!file_name.isEmpty());
	ui.text_yml_->setText("Please open and apply a yml file.");
}

void MainWindow::SlotApplyYML()
{
	if (parameter_)
	{
		delete parameter_;
	}

	parameter_ = new Parameter(ui.path_yml_->text().toStdString());
	//QMessageBox::information(this, "Information", "Parameters applied successfully!");

	QString temp;
	ui.value_fx_->setText(temp.setNum(parameter_->kCameraParameters_.fx_));
	ui.value_fy_->setText(temp.setNum(parameter_->kCameraParameters_.fy_));
	ui.value_cx_->setText(temp.setNum(parameter_->kCameraParameters_.cx_));
	ui.value_cy_->setText(temp.setNum(parameter_->kCameraParameters_.cy_));
	ui.value_scale_->setText(temp.setNum(parameter_->kCameraParameters_.scale_));
	ui.value_d0_->setText(temp.setNum(parameter_->kCameraParameters_.d0_));
	ui.value_d1_->setText(temp.setNum(parameter_->kCameraParameters_.d1_));
	ui.value_d2_->setText(temp.setNum(parameter_->kCameraParameters_.d2_));
	ui.value_d3_->setText(temp.setNum(parameter_->kCameraParameters_.d3_));
	ui.value_d4_->setText(temp.setNum(parameter_->kCameraParameters_.d4_));
	ui.value_fx_->setEnabled(true);
	ui.value_fy_->setEnabled(true);
	ui.value_cx_->setEnabled(true);
	ui.value_cy_->setEnabled(true);
	ui.value_scale_->setEnabled(true);
	ui.value_d0_->setEnabled(true);
	ui.value_d1_->setEnabled(true);
	ui.value_d2_->setEnabled(true);
	ui.value_d3_->setEnabled(true);
	ui.value_d4_->setEnabled(true);
	ui.button_modify_camera_->setEnabled(true);

	ui.text_yml_->setText("Parameters applied successfully!");
	ui.button_start_->setEnabled(true);
}

void MainWindow::SlotSelectVocabulary()
{
	QString file_name = QFileDialog::getOpenFileName(this, NULL, NULL, "*.txt");
	ui.path_vocabulary_->setText(file_name);
	ui.button_load_vocabulary_->setEnabled(!file_name.isEmpty());
	ui.text_vocabulary_->setText("Please open and load a bow vocabulary file.");
}

void MainWindow::SlotLoadVocabulary()
{
	if (bow_vocabulary_)
	{
		delete bow_vocabulary_;
	}

	bow_vocabulary_ = new DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>();
	ui.text_vocabulary_->setText("Loading BoW Vocabulary... UI will be stuck, please wait for a moment.");
	ui.text_vocabulary_->repaint();

	if (bow_vocabulary_->loadFromTextFile(ui.path_vocabulary_->text().toStdString()))
	{
		//QMessageBox::information(this, "Information", "BoW Vocabulary Loaded successfully!");
		ui.text_vocabulary_->setText("BoW vocabulary loaded successfully!");
	}
	else
	{
		//QMessageBox::warning(this, "Warning", "BoW Vocabulary Loaded failed!");
		ui.text_vocabulary_->setText("BoW vocabulary loaded failed, please try again.");
	}
}

void MainWindow::SlotModifyCamera()
{
	parameter_->kCameraParameters_.fx_ = ui.value_fx_->text().toFloat();
	parameter_->kCameraParameters_.fy_ = ui.value_fy_->text().toFloat();
	parameter_->kCameraParameters_.cx_ = ui.value_cx_->text().toFloat();
	parameter_->kCameraParameters_.cy_ = ui.value_cy_->text().toFloat();
	parameter_->kCameraParameters_.scale_ = ui.value_scale_->text().toFloat();
	parameter_->kCameraParameters_.d0_ = ui.value_d0_->text().toFloat();
	parameter_->kCameraParameters_.d1_ = ui.value_d1_->text().toFloat();
	parameter_->kCameraParameters_.d2_ = ui.value_d2_->text().toFloat();
	parameter_->kCameraParameters_.d3_ = ui.value_d3_->text().toFloat();
	parameter_->kCameraParameters_.d4_ = ui.value_d4_->text().toFloat();

	QMessageBox::information(this, "Information", "Camera parameters modified successfully!");
}

void MainWindow::SlotStartSLAM()
{
	system_ = new System();
	system_->SetQTWidget(ui.label_color_, ui.label_depth_, ui.qvtk_widget_, pcl_viewer_);
	system_->SetParameter(parameter_);
	system_->SetBoWVocabulary(bow_vocabulary_);

	system_thread_ = new std::thread(&System::Run, system_);
	vtk_timer_->start(1000);

	ui.button_start_->setEnabled(false);
	while (!system_->is_running_)
	{
		thread_sleep(1);
	}
	ui.button_stop_->setEnabled(true);
}

void MainWindow::SlotStopSLAM()
{
	system_->is_running_ = false;
	while (!system_->thread_over_)
	{
		thread_sleep(1);
	}
	system_thread_->join();
	
	delete system_thread_;
	vtk_timer_->stop();
	delete system_;

	ui.button_start_->setEnabled(true);
	ui.button_stop_->setEnabled(false);
}

void MainWindow::SlotUpdateVTK()
{
	if (system_->map_ != NULL)
	{
		if (system_->map_->vtk_flag_)
		{
			pcl_viewer_->removePointCloud();
			pcl_viewer_->addPointCloud<pcl::PointXYZRGBA>(system_->map_->global_cloud_);
			ui.qvtk_widget_->update();
			system_->map_->vtk_flag_ = false;
		}
	}
}
