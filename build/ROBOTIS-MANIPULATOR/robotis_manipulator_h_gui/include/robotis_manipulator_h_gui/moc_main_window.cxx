/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-MANIPULATOR/robotis_manipulator_h_gui/include/robotis_manipulator_h_gui/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_robotis_manipulator_h_gui__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      39,   38,   38,   38, 0x0a,
      72,   66,   38,   38, 0x0a,
     104,   66,   38,   38, 0x0a,
     135,   38,   38,   38, 0x0a,
     161,  155,   38,   38, 0x0a,
     215,  207,   38,   38, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_robotis_manipulator_h_gui__MainWindow[] = {
    "robotis_manipulator_h_gui::MainWindow\0"
    "\0on_actionAbout_triggered()\0check\0"
    "on_currpos_button_clicked(bool)\0"
    "on_despos_button_clicked(bool)\0"
    "updateLoggingView()\0x,y,z\0"
    "update_curr_pos_spinbox(double,double,double)\0"
    "x,y,z,w\0update_curr_ori_spinbox(double,double,double,double)\0"
};

void robotis_manipulator_h_gui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_currpos_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_despos_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->updateLoggingView(); break;
        case 4: _t->update_curr_pos_spinbox((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 5: _t->update_curr_ori_spinbox((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData robotis_manipulator_h_gui::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject robotis_manipulator_h_gui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_robotis_manipulator_h_gui__MainWindow,
      qt_meta_data_robotis_manipulator_h_gui__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &robotis_manipulator_h_gui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *robotis_manipulator_h_gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *robotis_manipulator_h_gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_robotis_manipulator_h_gui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int robotis_manipulator_h_gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
