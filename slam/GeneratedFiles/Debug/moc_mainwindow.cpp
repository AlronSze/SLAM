/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[15];
    char stringdata0[228];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 13), // "SlotSelectYML"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 11), // "SlotLoadYML"
QT_MOC_LITERAL(4, 38, 20), // "SlotSelectVocabulary"
QT_MOC_LITERAL(5, 59, 18), // "SlotLoadVocabulary"
QT_MOC_LITERAL(6, 78, 13), // "SlotStartSLAM"
QT_MOC_LITERAL(7, 92, 12), // "SlotStopSLAM"
QT_MOC_LITERAL(8, 105, 13), // "SlotUpdateVTK"
QT_MOC_LITERAL(9, 119, 16), // "SlotModifyCamera"
QT_MOC_LITERAL(10, 136, 17), // "SlotRefreshDevice"
QT_MOC_LITERAL(11, 154, 14), // "SlotOpenDevice"
QT_MOC_LITERAL(12, 169, 15), // "SlotCloseDevice"
QT_MOC_LITERAL(13, 185, 18), // "SlotEnterPhotoMode"
QT_MOC_LITERAL(14, 204, 23) // "SlotDestroypPhotoWindow"

    },
    "MainWindow\0SlotSelectYML\0\0SlotLoadYML\0"
    "SlotSelectVocabulary\0SlotLoadVocabulary\0"
    "SlotStartSLAM\0SlotStopSLAM\0SlotUpdateVTK\0"
    "SlotModifyCamera\0SlotRefreshDevice\0"
    "SlotOpenDevice\0SlotCloseDevice\0"
    "SlotEnterPhotoMode\0SlotDestroypPhotoWindow"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x0a /* Public */,
       3,    0,   80,    2, 0x0a /* Public */,
       4,    0,   81,    2, 0x0a /* Public */,
       5,    0,   82,    2, 0x0a /* Public */,
       6,    0,   83,    2, 0x0a /* Public */,
       7,    0,   84,    2, 0x0a /* Public */,
       8,    0,   85,    2, 0x0a /* Public */,
       9,    0,   86,    2, 0x0a /* Public */,
      10,    0,   87,    2, 0x0a /* Public */,
      11,    0,   88,    2, 0x0a /* Public */,
      12,    0,   89,    2, 0x0a /* Public */,
      13,    0,   90,    2, 0x0a /* Public */,
      14,    0,   91,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SlotSelectYML(); break;
        case 1: _t->SlotLoadYML(); break;
        case 2: _t->SlotSelectVocabulary(); break;
        case 3: _t->SlotLoadVocabulary(); break;
        case 4: _t->SlotStartSLAM(); break;
        case 5: _t->SlotStopSLAM(); break;
        case 6: _t->SlotUpdateVTK(); break;
        case 7: _t->SlotModifyCamera(); break;
        case 8: _t->SlotRefreshDevice(); break;
        case 9: _t->SlotOpenDevice(); break;
        case 10: _t->SlotCloseDevice(); break;
        case 11: _t->SlotEnterPhotoMode(); break;
        case 12: _t->SlotDestroypPhotoWindow(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
