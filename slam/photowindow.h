#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_photowindow.h"

#include <thread>

#include <OpenNI.h>

#include "inc/system.h"

class PhotoWindow : public QMainWindow
{
	Q_OBJECT

public:
	PhotoWindow(openni::VideoStream *p_color_stream, QWidget *p_parent = Q_NULLPTR);
	~PhotoWindow();

public slots:
	void SlotScreenshot();
	void SlotSelectPath();

private:
	void InitializeSlots();

private:
	Ui::PhotoWindowClass ui_;

	System *system_;
	std::thread *system_thread_;
};
