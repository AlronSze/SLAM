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
    QPushButton *button_apply_yml_;
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
    QLabel *label_14;
    QLineEdit *value_fx_;
    QLineEdit *value_fy_;
    QLineEdit *value_cx_;
    QLineEdit *value_cy_;
    QLineEdit *value_scale_;
    QLineEdit *value_d0_;
    QLineEdit *value_d1_;
    QLineEdit *value_d2_;
    QLineEdit *value_d3_;
    QLineEdit *value_d4_;
    QPushButton *button_modify_camera_;
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
    QMenuBar *menuBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindowClass)
    {
        if (MainWindowClass->objectName().isEmpty())
            MainWindowClass->setObjectName(QStringLiteral("MainWindowClass"));
        MainWindowClass->resize(1443, 911);
        centralWidget = new QWidget(MainWindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        button_select_yml_ = new QPushButton(centralWidget);
        button_select_yml_->setObjectName(QStringLiteral("button_select_yml_"));
        button_select_yml_->setGeometry(QRect(519, 70, 31, 23));
        path_yml_ = new QLineEdit(centralWidget);
        path_yml_->setObjectName(QStringLiteral("path_yml_"));
        path_yml_->setEnabled(false);
        path_yml_->setGeometry(QRect(110, 70, 401, 20));
        button_apply_yml_ = new QPushButton(centralWidget);
        button_apply_yml_->setObjectName(QStringLiteral("button_apply_yml_"));
        button_apply_yml_->setEnabled(false);
        button_apply_yml_->setGeometry(QRect(559, 70, 90, 23));
        path_vocabulary_ = new QLineEdit(centralWidget);
        path_vocabulary_->setObjectName(QStringLiteral("path_vocabulary_"));
        path_vocabulary_->setEnabled(false);
        path_vocabulary_->setGeometry(QRect(110, 490, 401, 20));
        button_select_vocabulary_ = new QPushButton(centralWidget);
        button_select_vocabulary_->setObjectName(QStringLiteral("button_select_vocabulary_"));
        button_select_vocabulary_->setGeometry(QRect(519, 490, 31, 23));
        button_load_vocabulary_ = new QPushButton(centralWidget);
        button_load_vocabulary_->setObjectName(QStringLiteral("button_load_vocabulary_"));
        button_load_vocabulary_->setEnabled(false);
        button_load_vocabulary_->setGeometry(QRect(559, 490, 90, 23));
        label_color_ = new QLabel(centralWidget);
        label_color_->setObjectName(QStringLiteral("label_color_"));
        label_color_->setGeometry(QRect(50, 610, 280, 210));
        QFont font;
        font.setPointSize(25);
        label_color_->setFont(font);
        label_color_->setFrameShape(QFrame::Box);
        label_color_->setAlignment(Qt::AlignCenter);
        label_depth_ = new QLabel(centralWidget);
        label_depth_->setObjectName(QStringLiteral("label_depth_"));
        label_depth_->setGeometry(QRect(370, 610, 280, 210));
        label_depth_->setFont(font);
        label_depth_->setFrameShape(QFrame::Box);
        label_depth_->setAlignment(Qt::AlignCenter);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(30, 30, 640, 400));
        QFont font1;
        font1.setPointSize(14);
        groupBox->setFont(font1);
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 40, 61, 21));
        QFont font2;
        font2.setPointSize(9);
        label->setFont(font2);
        text_yml_ = new QLabel(groupBox);
        text_yml_->setObjectName(QStringLiteral("text_yml_"));
        text_yml_->setGeometry(QRect(20, 80, 601, 16));
        text_yml_->setFont(font2);
        groupBox_6 = new QGroupBox(groupBox);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(20, 120, 260, 260));
        QFont font3;
        font3.setPointSize(12);
        groupBox_6->setFont(font3);
        label_3 = new QLabel(groupBox_6);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(30, 30, 20, 20));
        QFont font4;
        font4.setPointSize(10);
        label_3->setFont(font4);
        label_6 = new QLabel(groupBox_6);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(30, 65, 20, 20));
        label_6->setFont(font4);
        label_7 = new QLabel(groupBox_6);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(30, 100, 20, 20));
        label_7->setFont(font4);
        label_8 = new QLabel(groupBox_6);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(30, 135, 20, 20));
        label_8->setFont(font4);
        label_9 = new QLabel(groupBox_6);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(150, 30, 20, 20));
        label_9->setFont(font4);
        label_10 = new QLabel(groupBox_6);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(150, 65, 20, 20));
        label_10->setFont(font4);
        label_11 = new QLabel(groupBox_6);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(150, 100, 20, 20));
        label_11->setFont(font4);
        label_12 = new QLabel(groupBox_6);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(150, 135, 20, 20));
        label_12->setFont(font4);
        label_13 = new QLabel(groupBox_6);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(150, 170, 20, 20));
        label_13->setFont(font4);
        label_14 = new QLabel(groupBox_6);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(20, 170, 41, 20));
        label_14->setFont(font4);
        value_fx_ = new QLineEdit(groupBox_6);
        value_fx_->setObjectName(QStringLiteral("value_fx_"));
        value_fx_->setEnabled(false);
        value_fx_->setGeometry(QRect(70, 30, 60, 20));
        value_fx_->setFont(font4);
        value_fx_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_fy_ = new QLineEdit(groupBox_6);
        value_fy_->setObjectName(QStringLiteral("value_fy_"));
        value_fy_->setEnabled(false);
        value_fy_->setGeometry(QRect(70, 65, 60, 20));
        value_fy_->setFont(font4);
        value_fy_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_cx_ = new QLineEdit(groupBox_6);
        value_cx_->setObjectName(QStringLiteral("value_cx_"));
        value_cx_->setEnabled(false);
        value_cx_->setGeometry(QRect(70, 100, 60, 20));
        value_cx_->setFont(font4);
        value_cx_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_cy_ = new QLineEdit(groupBox_6);
        value_cy_->setObjectName(QStringLiteral("value_cy_"));
        value_cy_->setEnabled(false);
        value_cy_->setGeometry(QRect(70, 135, 60, 20));
        value_cy_->setFont(font4);
        value_cy_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_scale_ = new QLineEdit(groupBox_6);
        value_scale_->setObjectName(QStringLiteral("value_scale_"));
        value_scale_->setEnabled(false);
        value_scale_->setGeometry(QRect(70, 170, 60, 20));
        value_scale_->setFont(font4);
        value_scale_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d0_ = new QLineEdit(groupBox_6);
        value_d0_->setObjectName(QStringLiteral("value_d0_"));
        value_d0_->setEnabled(false);
        value_d0_->setGeometry(QRect(180, 30, 60, 20));
        value_d0_->setFont(font4);
        value_d0_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d1_ = new QLineEdit(groupBox_6);
        value_d1_->setObjectName(QStringLiteral("value_d1_"));
        value_d1_->setEnabled(false);
        value_d1_->setGeometry(QRect(180, 65, 60, 20));
        value_d1_->setFont(font4);
        value_d1_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d2_ = new QLineEdit(groupBox_6);
        value_d2_->setObjectName(QStringLiteral("value_d2_"));
        value_d2_->setEnabled(false);
        value_d2_->setGeometry(QRect(180, 100, 60, 20));
        value_d2_->setFont(font4);
        value_d2_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d3_ = new QLineEdit(groupBox_6);
        value_d3_->setObjectName(QStringLiteral("value_d3_"));
        value_d3_->setEnabled(false);
        value_d3_->setGeometry(QRect(180, 135, 60, 20));
        value_d3_->setFont(font4);
        value_d3_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        value_d4_ = new QLineEdit(groupBox_6);
        value_d4_->setObjectName(QStringLiteral("value_d4_"));
        value_d4_->setEnabled(false);
        value_d4_->setGeometry(QRect(180, 170, 60, 20));
        value_d4_->setFont(font4);
        value_d4_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        button_modify_camera_ = new QPushButton(groupBox_6);
        button_modify_camera_->setObjectName(QStringLiteral("button_modify_camera_"));
        button_modify_camera_->setEnabled(false);
        button_modify_camera_->setGeometry(QRect(20, 210, 220, 31));
        button_modify_camera_->setFont(font2);
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 450, 640, 110));
        groupBox_2->setFont(font1);
        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 40, 61, 20));
        label_2->setFont(font2);
        text_vocabulary_ = new QLabel(groupBox_2);
        text_vocabulary_->setObjectName(QStringLiteral("text_vocabulary_"));
        text_vocabulary_->setGeometry(QRect(20, 80, 601, 16));
        text_vocabulary_->setFont(font2);
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(690, 190, 720, 670));
        groupBox_3->setFont(font1);
        qvtk_widget_ = new QVTKWidget(groupBox_3);
        qvtk_widget_->setObjectName(QStringLiteral("qvtk_widget_"));
        qvtk_widget_->setGeometry(QRect(20, 30, 680, 620));
        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(30, 580, 641, 280));
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
        groupBox_5->setGeometry(QRect(690, 30, 720, 140));
        groupBox_5->setFont(font1);
        button_start_ = new QPushButton(groupBox_5);
        button_start_->setObjectName(QStringLiteral("button_start_"));
        button_start_->setEnabled(false);
        button_start_->setGeometry(QRect(450, 30, 211, 41));
        button_start_->setFont(font3);
        button_stop_ = new QPushButton(groupBox_5);
        button_stop_->setObjectName(QStringLiteral("button_stop_"));
        button_stop_->setEnabled(false);
        button_stop_->setGeometry(QRect(450, 80, 211, 41));
        button_stop_->setFont(font3);
        MainWindowClass->setCentralWidget(centralWidget);
        groupBox_5->raise();
        groupBox_4->raise();
        groupBox_3->raise();
        groupBox_2->raise();
        groupBox->raise();
        button_select_yml_->raise();
        path_yml_->raise();
        button_apply_yml_->raise();
        path_vocabulary_->raise();
        button_select_vocabulary_->raise();
        button_load_vocabulary_->raise();
        label_color_->raise();
        label_depth_->raise();
        menuBar = new QMenuBar(MainWindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1443, 23));
        MainWindowClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindowClass->setStatusBar(statusBar);

        retranslateUi(MainWindowClass);

        QMetaObject::connectSlotsByName(MainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowClass)
    {
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "SLAM System", Q_NULLPTR));
        button_select_yml_->setText(QApplication::translate("MainWindowClass", "...", Q_NULLPTR));
        button_apply_yml_->setText(QApplication::translate("MainWindowClass", "Apply", Q_NULLPTR));
        button_select_vocabulary_->setText(QApplication::translate("MainWindowClass", "...", Q_NULLPTR));
        button_load_vocabulary_->setText(QApplication::translate("MainWindowClass", "Load", Q_NULLPTR));
        label_color_->setText(QString());
        label_depth_->setText(QString());
        groupBox->setTitle(QApplication::translate("MainWindowClass", "YML Parameters", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindowClass", "YML File:", Q_NULLPTR));
        text_yml_->setText(QApplication::translate("MainWindowClass", "Please open and apply a yml file.", Q_NULLPTR));
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
        label_14->setText(QApplication::translate("MainWindowClass", "SCALE", Q_NULLPTR));
        button_modify_camera_->setText(QApplication::translate("MainWindowClass", "Modify Camera Parameters", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindowClass", "BoW Vocabulary", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindowClass", "Voc File:", Q_NULLPTR));
        text_vocabulary_->setText(QApplication::translate("MainWindowClass", "Please open and load a bow vocabulary file.", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("MainWindowClass", "PCL Viewer", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("MainWindowClass", "Image Viewer", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindowClass", "Color Image", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindowClass", "Depth Image", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("MainWindowClass", "SLAM Controller", Q_NULLPTR));
        button_start_->setText(QApplication::translate("MainWindowClass", "Start SLAM", Q_NULLPTR));
        button_stop_->setText(QApplication::translate("MainWindowClass", "Stop SLAM", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
