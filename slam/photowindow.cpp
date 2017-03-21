#include "photowindow.h"

#include <QFileDialog>

#ifndef SAFE_DELETE 
#define SAFE_DELETE(p) if(p) { delete (p); (p) = NULL; }
#endif

PhotoWindow::PhotoWindow(openni::VideoStream *p_color_stream, QWidget *p_parent) :
	QMainWindow(p_parent)
{
	ui_.setupUi(this);

	InitializeSlots();

	system_ = new System(p_color_stream);
	system_->SetQTDrawWidget(ui_.label_color_, ui_.text_screenshot_);

	system_thread_ = new std::thread(&System::RunPhotoMode, system_);
}

PhotoWindow::~PhotoWindow()
{
	system_->is_running_ = false;
	system_thread_->join();

	SAFE_DELETE(system_thread_);
	SAFE_DELETE(system_);
}

void PhotoWindow::InitializeSlots()
{
	connect(ui_.button_screenshot_, SIGNAL(clicked()), this, SLOT(SlotScreenshot()));
	connect(ui_.button_select_path_, SIGNAL(clicked()), this, SLOT(SlotSelectPath()));
}

void PhotoWindow::SlotScreenshot()
{
	system_->screenshot_flag_ = true;
}

void PhotoWindow::SlotSelectPath()
{
	QString path_name = QFileDialog::getExistingDirectory(this);

	ui_.path_save_->setText(path_name);

	if (path_name.isEmpty())
	{
		ui_.button_screenshot_->setEnabled(false);
		ui_.text_screenshot_->setText("Please select a path for saving screenshot.");
	}
	else
	{
		system_->screenshot_path_ = path_name.toStdString();
		ui_.button_screenshot_->setEnabled(true);
		ui_.text_screenshot_->setText("Your path has been selected!");
	}

}
