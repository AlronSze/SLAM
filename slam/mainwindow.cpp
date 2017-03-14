#include "mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include <string>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), parameter_(NULL),
	bow_vocabulary_(NULL), system_(NULL), color_stream_(NULL), depth_stream_(NULL),
	system_thread_(NULL), yml_flag_(false), voc_flag_(false), dev_flag_(false)
{
	ui.setupUi(this);

	InitializeSlots();
	InitializeVTK();
	InitializeOpenNI();
}

MainWindow::~MainWindow()
{
	if (parameter_)
	{
		delete parameter_;
	}
	if (bow_vocabulary_)
	{
		delete bow_vocabulary_;
	}
	if (system_thread_)
	{
		delete system_thread_;
	}
	if (system_)
	{
		delete system_;
	}
	if (color_stream_)
	{
		color_stream_->destroy();
		delete color_stream_;
	}
	if (depth_stream_)
	{
		depth_stream_->destroy();
		delete depth_stream_;
	}

	rgbd_device_.close();
	openni::OpenNI::shutdown();
}

void MainWindow::InitializeSlots()
{
	connect(ui.button_select_yml_, SIGNAL(clicked()), this, SLOT(SlotSelectYML()));
	connect(ui.button_apply_yml_, SIGNAL(clicked()), this, SLOT(SlotApplyYML()));
	connect(ui.button_select_vocabulary_, SIGNAL(clicked()), this, SLOT(SlotSelectVocabulary()));
	connect(ui.button_load_vocabulary_, SIGNAL(clicked()), this, SLOT(SlotLoadVocabulary()));
	connect(ui.button_start_, SIGNAL(clicked()), this, SLOT(SlotStartSLAM()));
	connect(ui.button_stop_, SIGNAL(clicked()), this, SLOT(SlotStopSLAM()));
	connect(ui.button_modify_camera_, SIGNAL(clicked()), this, SLOT(SlotModifyCamera()));
	connect(ui.button_refresh_device_, SIGNAL(clicked()), this, SLOT(SlotRefreshDevice()));
	connect(ui.button_open_device_, SIGNAL(clicked()), this, SLOT(SlotOpenDevice()));
	connect(ui.button_close_device_, SIGNAL(clicked()), this, SLOT(SlotCloseDevice()));
}

void MainWindow::InitializeVTK()
{
	pcl_viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	vtkSmartPointer<vtkRenderWindow> render_window = pcl_viewer_->getRenderWindow();
	ui.qvtk_widget_->SetRenderWindow(render_window);

	pcl_viewer_->setShowFPS(false);
	pcl_viewer_->addCoordinateSystem(0.1);
	pcl_viewer_->setCameraPosition(0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
	ui.qvtk_widget_->update();

	connect(&vtk_timer_, SIGNAL(timeout()), this, SLOT(SlotUpdateVTK()));
}

void MainWindow::InitializeOpenNI()
{
	openni::OpenNI::initialize();
	SlotRefreshDevice();
	color_stream_ = new openni::VideoStream();
	depth_stream_ = new openni::VideoStream();
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
	ui.button_modify_camera_->setEnabled(true);

	yml_flag_ = true;
	ui.text_yml_->setText("Parameters applied successfully!");
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
	ui.text_vocabulary_->setText("Loading BoW vocabulary... UI will be stuck, please wait for a moment.");
	ui.text_vocabulary_->repaint();

	if (bow_vocabulary_->loadFromTextFile(ui.path_vocabulary_->text().toStdString()))
	{
		voc_flag_ = true;
		//QMessageBox::information(this, "Information", "BoW Vocabulary Loaded successfully!");
		ui.text_vocabulary_->setText("BoW vocabulary loaded successfully!");
	}
	else
	{
		voc_flag_ = false;
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
	if (!dev_flag_)
	{
		QMessageBox::warning(this, "Warning", "Please open device before starting SLAM!");
		return;
	}
	if (!yml_flag_)
	{
		QMessageBox::warning(this, "Warning", "Please apply parameters before starting SLAM!");
		return;
	}
	if (!voc_flag_)
	{
		//bow_vocabulary_ = new DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>();
		QMessageBox::warning(this, "Warning", "Please load vocabulary before starting SLAM!");
		return;
	}

	ui.button_start_->setEnabled(false);
	ui.button_close_device_->setEnabled(false);
	ui.button_select_yml_->setEnabled(false);
	ui.button_apply_yml_->setEnabled(false);
	ui.button_modify_camera_->setEnabled(false);
	ui.button_select_vocabulary_->setEnabled(false);
	ui.button_load_vocabulary_->setEnabled(false);

	system_ = new System(color_stream_, depth_stream_);
	system_->SetQTDrawWidget(ui.label_color_, ui.label_depth_, ui.qvtk_widget_, pcl_viewer_);
	system_->SetQTStatusWidget(ui.value_track_status_, ui.value_frame_count_, ui.value_keyframe_count_, ui.value_loop_count_);
	system_->SetParameter(parameter_);
	system_->SetBoWVocabulary(bow_vocabulary_);

	system_thread_ = new std::thread(&System::Run, system_);
	vtk_timer_.start(1000);

	ui.button_stop_->setEnabled(true);
}

void MainWindow::SlotStopSLAM()
{
	ui.button_stop_->setEnabled(false);

	system_->is_running_ = false;
	system_thread_->join();
	delete system_thread_;
	system_thread_ = NULL;

	vtk_timer_.stop();
	SlotUpdateVTK();

	delete system_;
	system_ = NULL;

	ui.button_start_->setEnabled(true);
	ui.button_close_device_->setEnabled(true);
	ui.button_select_yml_->setEnabled(true);
	ui.button_apply_yml_->setEnabled(true);
	ui.button_modify_camera_->setEnabled(true);
	ui.button_select_vocabulary_->setEnabled(true);
	ui.button_load_vocabulary_->setEnabled(true);
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

void MainWindow::SlotRefreshDevice()
{
	ui.box_device_->clear();
	openni::OpenNI::enumerateDevices(&rgbd_device_list_);
	for (int32_t i = 0; i < rgbd_device_list_.getSize(); ++i)
	{
		const openni::DeviceInfo & device_info = rgbd_device_list_[i];
		ui.box_device_->addItem(device_info.getName() + tr(": ") + device_info.getUri());
	}
}

void MainWindow::SlotOpenDevice()
{
	int32_t device_index = ui.box_device_->currentIndex();

	ui.text_device_->setText("Opening RGB-D device... UI will be stuck, please wait for a moment.");
	ui.text_device_->repaint();

	if (rgbd_device_.open(rgbd_device_list_[device_index].getUri()) == openni::STATUS_OK)
	{
		color_stream_->create(rgbd_device_, openni::SENSOR_COLOR);
		depth_stream_->create(rgbd_device_, openni::SENSOR_DEPTH);

		openni::VideoMode color_mode;
		color_mode.setResolution(640, 480);
		color_mode.setFps(30);
		color_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
		color_stream_->setVideoMode(color_mode);

		openni::VideoMode depth_mode;
		depth_mode.setResolution(640, 480);
		depth_mode.setFps(30);
		depth_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		depth_stream_->setVideoMode(depth_mode);

		if (rgbd_device_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			rgbd_device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}

		color_stream_->start();
		depth_stream_->start();
		color_stream_->stop();
		depth_stream_->stop();

		dev_flag_ = true;
		ui.text_device_->setText("RGB-D device opened successfully!");
		ui.button_open_device_->setEnabled(false);
		ui.button_close_device_->setEnabled(true);
	}
	else
	{
		ui.text_device_->setText("RGB-D device opened failed, please try again.");
	}
}

void MainWindow::SlotCloseDevice()
{
	color_stream_->destroy();
	depth_stream_->destroy();
	rgbd_device_.close();

	dev_flag_ = false;
	ui.text_device_->setText("RGB-D device closed successfully!");
	ui.button_open_device_->setEnabled(true);
	ui.button_close_device_->setEnabled(false);
}
