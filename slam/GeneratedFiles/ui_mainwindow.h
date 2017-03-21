/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindowClass
{
public:
    QWidget *centralWidget;
    QPushButton *button_select_yml_;
    QLineEdit *path_yml_;
    QPushButton *button_load_yml_;
    QLineEdit *path_vocabulary_;
    QPushButton *button_select_vocabulary_;
    QPushButton *button_load_vocabulary_;
    QLabel *label_color_;
    QLabel *label_depth_;
    QGroupBox *groupBox;
    QLabel *label;
    QLabel *text_yml_;
    QGroupBox *groupBox_6;
    QLabel *label_3;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *value_fx_;
    QLineEdit *value_fy_;
    QLineEdit *value_cx_;
    QLineEdit *value_cy_;
    QLineEdit *value_d0_;
    QLineEdit *value_d1_;
    QLineEdit *value_d2_;
    QLineEdit *value_d3_;
    QLineEdit *value_d4_;
    QPushButton *button_modify_camera_;
    QLabel *label_14;
    QLineEdit *value_scale_;
    QLabel *label_20;
    QLineEdit *value_max_dist_;
    QGroupBox *groupBox_2;
    QLabel *label_2;
    QLabel *text_vocabulary_;
    QGroupBox *groupBox_3;
    QVTKWidget *qvtk_widget_;
    QGroupBox *groupBox_4;
    QLabel *label_4;
    QLabel *label_5;
    QGroupBox *groupBox_5;
    QPushButton *button_start_;
    QPushButton *button_stop_;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLineEdit *value_track_status_;
    QLineEdit *value_frame_count_;
    QLineEdit *value_keyframe_count_;
    QLabel *label_19;
    QLineEdit *value_loop_count_;
    QGroupBox *groupBox_7;
    QComboBox *box_device_;
    QLabel *label_15;
    QPushButton *button_refresh_device_;
    QPushButton *button_open_device_;
    QPushButton *button_close_device_;
    QLabel *text_device_;
    QPushButton *button_photo_mode_;
    QStatusBar *statusBar;
    QMenuBar *menuBar;

    void setupUi(QMainWindow *MainWindowClass)
    {
        if (MainWindowClass->objectName().isEmpty())
            MainWindowClass->setObjectName(QStringLiteral("MainWindowClass"));
        MainWindowClass->setEnabled(true);
        MainWindowClass->resize(1440, 905);
        centralWidget = new QWidget(MainWindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        button_select_yml_ = new QPushButton(centralWidget);
        button_select_yml_->setObjectName(QStringLiteral("button_select_yml_"));
        button_select_yml_->setGeometry(QRect(520, 190, 30, 23));
        path_yml_ = new QLineEdit(centralWidget);
        path_yml_->setObjectName(QStringLiteral("path_yml_"));
        path_yml_->setEnabled(false);
        path_yml_->setGeometry(QRect(110, 190, 400, 23));
        path_yml_->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        button_load_yml_ = new QPushButton(centralWidget);
        button_load_yml_->setObjectName(QStringLiteral("button_load_yml_"));
        button_load_yml_->setEnabled(true);
        button_load_yml_->setGeometry(QRect(560, 190, 90, 23));
        path_vocabulary_ = new QLineEdit(centralWidget);
        path_vocabulary_->setObjectName(QStringLiteral("path_vocabulary_"));
        path_vocabulary_->setEnabled(false);
        path_vocabulary_->setGeometry(QRect(110, 490, 400, 23));
        button_select_vocabulary_ = new QPushButton(centralWidget);
        button_select_vocabulary_->setObjectName(QStringLiteral("button_select_vocabulary_"));
        button_select_vocabulary_->setGeometry(QRect(520, 490, 30, 23));
        button_load_vocabulary_ = new QPushButton(centralWidget);
        button_load_vocabulary_->setObjectName(QStringLiteral("button_load_vocabulary_"));
        button_load_vocabulary_->setEnabled(true);
        button_load_vocabulary_->setGeometry(QRect(560, 490, 90, 23));
        label_color_ = new QLabel(centralWidget);
        label_color_->setObjectName(QStringLiteral("label_color_"));
        label_color_->setGeometry(QRect(50, 610, 280, 210));
        QFont font;
        font.setPointSize(25);
        label_color_->setFont(font);
        label_color_->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        label_color_->setFrameShape(QFrame::Box);
        label_color_->setAlignment(Qt::AlignCenter);
        label_depth_ = new QLabel(centralWidget);
        label_depth_->setObjectName(QStringLiteral("label_depth_"));
        label_depth_->setGeometry(QRect(370, 610, 280, 210));
        label_depth_->setFont(font);
        label_depth_->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        label_depth_->setFrameShape(QFrame::Box);
        label_depth_->setAlignment(Qt::AlignCenter);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(30, 150, 640, 280));
        QFont font1;
        font1.setPointSize(14);
        groupBox->setFont(font1);
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 40, 60, 23));
        QFont font2;
        font2.setPointSize(9);
        label->setFont(font2);
        text_yml_ = new QLabel(groupBox);
        text_yml_->setObjectName(QStringLiteral("text_yml_"));
        text_yml_->setGeometry(QRect(20, 80, 600, 16));
        text_yml_->setFont(font2);
        groupBox_6 = new QGroupBox(groupBox);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(20, 110, 600, 150));
        QFont font3;
        font3.setPointSize(12);
        groupBox_6->setFont(font3);
        label_3 = new QLabel(groupBox_6);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 40, 50, 20));
        QFont font4;
        font4.setPointSize(10);
        label_3->setFont(font4);
        label_3->setAlignment(Qt::AlignCenter);
        label_6 = new QLabel(groupBox_6);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(20, 75, 50, 20));
        label_6->setFont(font4);
        label_6->setAlignment(Qt::AlignCenter);
        label_7 = new QLabel(groupBox_6);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(160, 75, 50, 20));
        label_7->setFont(font4);
        label_7->setAlignment(Qt::AlignCenter);
        label_8 = new QLabel(groupBox_6);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(160, 40, 50, 20));
        label_8->setFont(font4);
        label_8->setAlignment(Qt::AlignCenter);
        label_9 = new QLabel(groupBox_6);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(300, 40, 50, 20));
        label_9->setFont(font4);
        label_9->setAlignment(Qt::AlignCenter);
        label_10 = new QLabel(groupBox_6);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(300, 75, 50, 20));
        label_10->setFont(font4);
        label_10->setAlignment(Qt::AlignCenter);
        label_11 = new QLabel(groupBox_6);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(300, 110, 50, 20));
        label_11->setFont(font4);
        label_11->setAlignment(Qt::AlignCenter);
        label_12 = new QLabel(groupBox_6);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(440, 40, 50, 20));
        label_12->setFont(font4);
        label_12->setAlignment(Qt::AlignCenter);
        label_13 = new QLabel(groupBox_6);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(440, 75, 50, 20));
        label_13->setFont(font4);
        label_13->setAlignment(Qt::AlignCenter);
        value_fx_ = new QLineEdit(groupBox_6);
        value_fx_->setObjectName(QStringLiteral("value_fx_"));
        value_fx_->setEnabled(true);
        value_fx_->setGeometry(QRect(80, 40, 70, 20));
        value_fx_->setFont(font4);
        value_fx_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_fy_ = new QLineEdit(groupBox_6);
        value_fy_->setObjectName(QStringLiteral("value_fy_"));
        value_fy_->setEnabled(true);
        value_fy_->setGeometry(QRect(80, 75, 70, 20));
        value_fy_->setFont(font4);
        value_fy_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_cx_ = new QLineEdit(groupBox_6);
        value_cx_->setObjectName(QStringLiteral("value_cx_"));
        value_cx_->setEnabled(true);
        value_cx_->setGeometry(QRect(220, 75, 70, 20));
        value_cx_->setFont(font4);
        value_cx_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_cy_ = new QLineEdit(groupBox_6);
        value_cy_->setObjectName(QStringLiteral("value_cy_"));
        value_cy_->setEnabled(true);
        value_cy_->setGeometry(QRect(220, 40, 70, 20));
        value_cy_->setFont(font4);
        value_cy_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d0_ = new QLineEdit(groupBox_6);
        value_d0_->setObjectName(QStringLiteral("value_d0_"));
        value_d0_->setEnabled(true);
        value_d0_->setGeometry(QRect(360, 40, 70, 20));
        value_d0_->setFont(font4);
        value_d0_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d1_ = new QLineEdit(groupBox_6);
        value_d1_->setObjectName(QStringLiteral("value_d1_"));
        value_d1_->setEnabled(true);
        value_d1_->setGeometry(QRect(360, 75, 70, 20));
        value_d1_->setFont(font4);
        value_d1_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d2_ = new QLineEdit(groupBox_6);
        value_d2_->setObjectName(QStringLiteral("value_d2_"));
        value_d2_->setEnabled(true);
        value_d2_->setGeometry(QRect(360, 110, 70, 20));
        value_d2_->setFont(font4);
        value_d2_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d3_ = new QLineEdit(groupBox_6);
        value_d3_->setObjectName(QStringLiteral("value_d3_"));
        value_d3_->setEnabled(true);
        value_d3_->setGeometry(QRect(500, 40, 70, 20));
        value_d3_->setFont(font4);
        value_d3_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d4_ = new QLineEdit(groupBox_6);
        value_d4_->setObjectName(QStringLiteral("value_d4_"));
        value_d4_->setEnabled(true);
        value_d4_->setGeometry(QRect(500, 75, 70, 20));
        value_d4_->setFont(font4);
        value_d4_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        button_modify_camera_ = new QPushButton(groupBox_6);
        button_modify_camera_->setObjectName(QStringLiteral("button_modify_camera_"));
        button_modify_camera_->setEnabled(false);
        button_modify_camera_->setGeometry(QRect(455, 108, 120, 24));
        button_modify_camera_->setFont(font2);
        label_14 = new QLabel(groupBox_6);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(20, 110, 50, 20));
        label_14->setFont(font4);
        label_14->setAlignment(Qt::AlignCenter);
        value_scale_ = new QLineEdit(groupBox_6);
        value_scale_->setObjectName(QStringLiteral("value_scale_"));
        value_scale_->setEnabled(true);
        value_scale_->setGeometry(QRect(80, 110, 70, 20));
        value_scale_->setFont(font4);
        value_scale_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_20 = new QLabel(groupBox_6);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(160, 110, 50, 20));
        label_20->setFont(font4);
        label_20->setAlignment(Qt::AlignCenter);
        value_max_dist_ = new QLineEdit(groupBox_6);
        value_max_dist_->setObjectName(QStringLiteral("value_max_dist_"));
        value_max_dist_->setEnabled(true);
        value_max_dist_->setGeometry(QRect(220, 110, 70, 20));
        value_max_dist_->setFont(font4);
        value_max_dist_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 450, 640, 110));
        groupBox_2->setFont(font1);
        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 40, 60, 23));
        label_2->setFont(font2);
        text_vocabulary_ = new QLabel(groupBox_2);
        text_vocabulary_->setObjectName(QStringLiteral("text_vocabulary_"));
        text_vocabulary_->setGeometry(QRect(20, 80, 600, 16));
        text_vocabulary_->setFont(font2);
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(690, 170, 720, 690));
        groupBox_3->setFont(font1);
        qvtk_widget_ = new QVTKWidget(groupBox_3);
        qvtk_widget_->setObjectName(QStringLiteral("qvtk_widget_"));
        qvtk_widget_->setGeometry(QRect(20, 30, 680, 640));
        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(30, 580, 640, 280));
        groupBox_4->setFont(font1);
        label_4 = new QLabel(groupBox_4);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(20, 244, 280, 30));
        label_4->setFont(font4);
        label_4->setAlignment(Qt::AlignCenter);
        label_5 = new QLabel(groupBox_4);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(340, 244, 280, 30));
        label_5->setFont(font4);
        label_5->setAlignment(Qt::AlignCenter);
        groupBox_5 = new QGroupBox(centralWidget);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(690, 20, 720, 130));
        groupBox_5->setFont(font1);
        button_start_ = new QPushButton(groupBox_5);
        button_start_->setObjectName(QStringLiteral("button_start_"));
        button_start_->setEnabled(true);
        button_start_->setGeometry(QRect(560, 35, 140, 30));
        button_start_->setFont(font3);
        button_stop_ = new QPushButton(groupBox_5);
        button_stop_->setObjectName(QStringLiteral("button_stop_"));
        button_stop_->setEnabled(false);
        button_stop_->setGeometry(QRect(560, 75, 140, 30));
        button_stop_->setFont(font3);
        label_16 = new QLabel(groupBox_5);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(20, 40, 110, 23));
        label_16->setFont(font2);
        label_17 = new QLabel(groupBox_5);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(20, 80, 110, 23));
        label_17->setFont(font2);
        label_18 = new QLabel(groupBox_5);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(280, 80, 120, 23));
        label_18->setFont(font2);
        value_track_status_ = new QLineEdit(groupBox_5);
        value_track_status_->setObjectName(QStringLiteral("value_track_status_"));
        value_track_status_->setEnabled(false);
        value_track_status_->setGeometry(QRect(130, 40, 110, 23));
        value_track_status_->setFont(font2);
        value_track_status_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_frame_count_ = new QLineEdit(groupBox_5);
        value_frame_count_->setObjectName(QStringLiteral("value_frame_count_"));
        value_frame_count_->setEnabled(false);
        value_frame_count_->setGeometry(QRect(130, 80, 110, 23));
        value_frame_count_->setFont(font2);
        value_frame_count_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_keyframe_count_ = new QLineEdit(groupBox_5);
        value_keyframe_count_->setObjectName(QStringLiteral("value_keyframe_count_"));
        value_keyframe_count_->setEnabled(false);
        value_keyframe_count_->setGeometry(QRect(400, 80, 110, 23));
        value_keyframe_count_->setFont(font2);
        value_keyframe_count_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_19 = new QLabel(groupBox_5);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(280, 40, 120, 23));
        label_19->setFont(font2);
        value_loop_count_ = new QLineEdit(groupBox_5);
        value_loop_count_->setObjectName(QStringLiteral("value_loop_count_"));
        value_loop_count_->setEnabled(false);
        value_loop_count_->setGeometry(QRect(400, 40, 110, 23));
        value_loop_count_->setFont(font2);
        value_loop_count_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_7 = new QGroupBox(centralWidget);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        groupBox_7->setGeometry(QRect(30, 20, 640, 110));
        groupBox_7->setFont(font1);
        box_device_ = new QComboBox(groupBox_7);
        box_device_->setObjectName(QStringLiteral("box_device_"));
        box_device_->setGeometry(QRect(100, 40, 300, 23));
        box_device_->setFont(font2);
        label_15 = new QLabel(groupBox_7);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(20, 40, 80, 23));
        label_15->setFont(font2);
        button_refresh_device_ = new QPushButton(groupBox_7);
        button_refresh_device_->setObjectName(QStringLiteral("button_refresh_device_"));
        button_refresh_device_->setGeometry(QRect(420, 40, 60, 23));
        button_refresh_device_->setFont(font2);
        button_open_device_ = new QPushButton(groupBox_7);
        button_open_device_->setObjectName(QStringLiteral("button_open_device_"));
        button_open_device_->setGeometry(QRect(490, 40, 60, 23));
        button_open_device_->setFont(font2);
        button_close_device_ = new QPushButton(groupBox_7);
        button_close_device_->setObjectName(QStringLiteral("button_close_device_"));
        button_close_device_->setEnabled(false);
        button_close_device_->setGeometry(QRect(560, 40, 60, 23));
        button_close_device_->setFont(font2);
        text_device_ = new QLabel(groupBox_7);
        text_device_->setObjectName(QStringLiteral("text_device_"));
        text_device_->setGeometry(QRect(20, 80, 401, 16));
        text_device_->setFont(font2);
        button_photo_mode_ = new QPushButton(groupBox_7);
        button_photo_mode_->setObjectName(QStringLiteral("button_photo_mode_"));
        button_photo_mode_->setEnabled(false);
        button_photo_mode_->setGeometry(QRect(420, 75, 200, 23));
        button_photo_mode_->setFont(font2);
        MainWindowClass->setCentralWidget(centralWidget);
        groupBox_5->raise();
        groupBox_4->raise();
        groupBox_3->raise();
        groupBox_2->raise();
        groupBox->raise();
        path_yml_->raise();
        button_load_yml_->raise();
        path_vocabulary_->raise();
        button_select_vocabulary_->raise();
        button_load_vocabulary_->raise();
        label_color_->raise();
        label_depth_->raise();
        groupBox_7->raise();
        button_select_yml_->raise();
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindowClass->setStatusBar(statusBar);
        menuBar = new QMenuBar(MainWindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1440, 23));
        MainWindowClass->setMenuBar(menuBar);

        retranslateUi(MainWindowClass);

        QMetaObject::connectSlotsByName(MainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowClass)
    {
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "SLAM System", Q_NULLPTR));
        button_select_yml_->setText(QApplication::translate("MainWindowClass", "...", Q_NULLPTR));
        path_yml_->setText(QApplication::translate("MainWindowClass", "./yml/parameter.yml", Q_NULLPTR));
        button_load_yml_->setText(QApplication::translate("MainWindowClass", "Load", Q_NULLPTR));
        path_vocabulary_->setText(QApplication::translate("MainWindowClass", "./vocabulary/ORBvoc.txt", Q_NULLPTR));
        button_select_vocabulary_->setText(QApplication::translate("MainWindowClass", "...", Q_NULLPTR));
        button_load_vocabulary_->setText(QApplication::translate("MainWindowClass", "Load", Q_NULLPTR));
        label_color_->setText(QString());
        label_depth_->setText(QString());
        groupBox->setTitle(QApplication::translate("MainWindowClass", "YML Parameters", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindowClass", "YML File:", Q_NULLPTR));
        text_yml_->setText(QApplication::translate("MainWindowClass", "Please open and Load a YML file.", Q_NULLPTR));
        groupBox_6->setTitle(QApplication::translate("MainWindowClass", "Camera Parameters", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindowClass", "FX", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindowClass", "FY", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindowClass", "CX", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindowClass", "CY", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindowClass", "D0", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindowClass", "D1", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindowClass", "D2", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindowClass", "D3", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindowClass", "D4", Q_NULLPTR));
        button_modify_camera_->setText(QApplication::translate("MainWindowClass", "Modify Parameters", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindowClass", "SCALE", Q_NULLPTR));
        label_20->setText(QApplication::translate("MainWindowClass", "MAXDIST", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindowClass", "BoW Vocabulary", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindowClass", "Voc File:", Q_NULLPTR));
        text_vocabulary_->setText(QApplication::translate("MainWindowClass", "Please open and load a BoW vocabulary file.", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("MainWindowClass", "PCL Viewer", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("MainWindowClass", "Image Viewer", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindowClass", "Color Image", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindowClass", "Depth Image", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("MainWindowClass", "SLAM Controller", Q_NULLPTR));
        button_start_->setText(QApplication::translate("MainWindowClass", "Start SLAM", Q_NULLPTR));
        button_stop_->setText(QApplication::translate("MainWindowClass", "Stop SLAM", Q_NULLPTR));
        label_16->setText(QApplication::translate("MainWindowClass", "Tracking Status:", Q_NULLPTR));
        label_17->setText(QApplication::translate("MainWindowClass", "Frame Number:", Q_NULLPTR));
        label_18->setText(QApplication::translate("MainWindowClass", "Key Frame Number:", Q_NULLPTR));
        value_track_status_->setText(QApplication::translate("MainWindowClass", "Stop", Q_NULLPTR));
        value_frame_count_->setText(QApplication::translate("MainWindowClass", "0", Q_NULLPTR));
        value_keyframe_count_->setText(QApplication::translate("MainWindowClass", "0", Q_NULLPTR));
        label_19->setText(QApplication::translate("MainWindowClass", "Global Loop Number: ", Q_NULLPTR));
        value_loop_count_->setText(QApplication::translate("MainWindowClass", "0", Q_NULLPTR));
        groupBox_7->setTitle(QApplication::translate("MainWindowClass", "RGB-D Device", Q_NULLPTR));
        label_15->setText(QApplication::translate("MainWindowClass", "Device List:", Q_NULLPTR));
        button_refresh_device_->setText(QApplication::translate("MainWindowClass", "Refresh", Q_NULLPTR));
        button_open_device_->setText(QApplication::translate("MainWindowClass", "Open", Q_NULLPTR));
        button_close_device_->setText(QApplication::translate("MainWindowClass", "Close", Q_NULLPTR));
        text_device_->setText(QApplication::translate("MainWindowClass", "Please open a RGB-D device.", Q_NULLPTR));
        button_photo_mode_->setText(QApplication::translate("MainWindowClass", "Enter Photo Mode", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
