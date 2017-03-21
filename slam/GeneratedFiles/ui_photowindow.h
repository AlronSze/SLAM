/********************************************************************************
** Form generated from reading UI file 'photowindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PHOTOWINDOW_H
#define UI_PHOTOWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PhotoWindowClass
{
public:
    QWidget *centralwidget;
    QGroupBox *groupBox;
    QLabel *label_color_;
    QGroupBox *groupBox_2;
    QLineEdit *path_save_;
    QPushButton *button_screenshot_;
    QPushButton *button_select_path_;
    QLabel *text_screenshot_;
    QLabel *label_3;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *PhotoWindowClass)
    {
        if (PhotoWindowClass->objectName().isEmpty())
            PhotoWindowClass->setObjectName(QStringLiteral("PhotoWindowClass"));
        PhotoWindowClass->resize(740, 710);
        centralwidget = new QWidget(PhotoWindowClass);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(30, 20, 680, 530));
        QFont font;
        font.setPointSize(14);
        groupBox->setFont(font);
        label_color_ = new QLabel(groupBox);
        label_color_->setObjectName(QStringLiteral("label_color_"));
        label_color_->setGeometry(QRect(20, 30, 640, 480));
        label_color_->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 570, 681, 111));
        groupBox_2->setFont(font);
        path_save_ = new QLineEdit(groupBox_2);
        path_save_->setObjectName(QStringLiteral("path_save_"));
        path_save_->setEnabled(false);
        path_save_->setGeometry(QRect(80, 40, 410, 23));
        QFont font1;
        font1.setPointSize(9);
        path_save_->setFont(font1);
        button_screenshot_ = new QPushButton(groupBox_2);
        button_screenshot_->setObjectName(QStringLiteral("button_screenshot_"));
        button_screenshot_->setEnabled(false);
        button_screenshot_->setGeometry(QRect(540, 40, 120, 23));
        button_screenshot_->setFont(font1);
        button_select_path_ = new QPushButton(groupBox_2);
        button_select_path_->setObjectName(QStringLiteral("button_select_path_"));
        button_select_path_->setGeometry(QRect(500, 40, 30, 23));
        button_select_path_->setFont(font1);
        text_screenshot_ = new QLabel(groupBox_2);
        text_screenshot_->setObjectName(QStringLiteral("text_screenshot_"));
        text_screenshot_->setGeometry(QRect(20, 80, 470, 16));
        text_screenshot_->setFont(font1);
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 40, 60, 23));
        label_3->setFont(font1);
        PhotoWindowClass->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(PhotoWindowClass);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        PhotoWindowClass->setStatusBar(statusbar);

        retranslateUi(PhotoWindowClass);

        QMetaObject::connectSlotsByName(PhotoWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *PhotoWindowClass)
    {
        PhotoWindowClass->setWindowTitle(QApplication::translate("PhotoWindowClass", "Photo Mode", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("PhotoWindowClass", "Image Viewer", Q_NULLPTR));
        label_color_->setText(QString());
        groupBox_2->setTitle(QApplication::translate("PhotoWindowClass", "Controller", Q_NULLPTR));
        path_save_->setText(QString());
        button_screenshot_->setText(QApplication::translate("PhotoWindowClass", "Screenshot", Q_NULLPTR));
        button_select_path_->setText(QApplication::translate("PhotoWindowClass", "...", Q_NULLPTR));
        text_screenshot_->setText(QApplication::translate("PhotoWindowClass", "Please select a path for saving photos.", Q_NULLPTR));
        label_3->setText(QApplication::translate("PhotoWindowClass", "Save DIR:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PhotoWindowClass: public Ui_PhotoWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PHOTOWINDOW_H
