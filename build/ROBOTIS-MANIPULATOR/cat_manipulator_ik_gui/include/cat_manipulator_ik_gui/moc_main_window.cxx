/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-MANIPULATOR/cat_manipulator_ik_gui/include/cat_manipulator_ik_gui/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cat_manipulator_ik_gui__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      36,   35,   35,   35, 0x0a,
      69,   63,   35,   35, 0x0a,
     107,   63,   35,   35, 0x0a,
     138,   63,   35,   35, 0x0a,
     173,   63,   35,   35, 0x0a,
     211,   63,   35,   35, 0x0a,
     242,   63,   35,   35, 0x0a,
     284,   63,   35,   35, 0x0a,
     322,   63,   35,   35, 0x0a,
     360,   63,   35,   35, 0x0a,
     398,   63,   35,   35, 0x0a,
     436,   63,   35,   35, 0x0a,
     470,   63,   35,   35, 0x0a,
     505,   63,   35,   35, 0x0a,
     540,   63,   35,   35, 0x0a,
     579,   63,   35,   35, 0x0a,
     624,   63,   35,   35, 0x0a,
     669,   63,   35,   35, 0x0a,
     706,   63,   35,   35, 0x0a,
     742,   35,   35,   35, 0x0a,
     762,   35,   35,   35, 0x0a,
     793,  790,   35,   35, 0x0a,
     821,   35,   35,   35, 0x0a,
     858,  841,   35,   35, 0x0a,
     907,  903,   35,   35, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_cat_manipulator_ik_gui__MainWindow[] = {
    "cat_manipulator_ik_gui::MainWindow\0\0"
    "on_actionAbout_triggered()\0check\0"
    "on_getJointState_button_clicked(bool)\0"
    "on_getEEF_button_clicked(bool)\0"
    "on_getTagPose_button_clicked(bool)\0"
    "on_setJointState_button_clicked(bool)\0"
    "on_setEEF_button_clicked(bool)\0"
    "on_publishJointState_button_clicked(bool)\0"
    "on_move_gotoRest_button_clicked(bool)\0"
    "on_move_gotoPos1_button_clicked(bool)\0"
    "on_move_gotoPos2_button_clicked(bool)\0"
    "on_move_gotoPos3_button_clicked(bool)\0"
    "on_move_drop_button_clicked(bool)\0"
    "on_move_pickX_button_clicked(bool)\0"
    "on_move_pickY_button_clicked(bool)\0"
    "on_move_pickYback_button_clicked(bool)\0"
    "on_move_gobackdroprobot_button_clicked(bool)\0"
    "on_move_gobackdroptable_button_clicked(bool)\0"
    "on_closegripper_button_clicked(bool)\0"
    "on_opengripper_button_clicked(bool)\0"
    "updateEEFinWindow()\0updateJointStatesinWindow()\0"
    "js\0sendJoints(Eigen::VectorXd)\0"
    "updateLoggingView()\0cur_joint_states\0"
    "update_joint_states_spinbox(Eigen::VectorXd)\0"
    "eef\0update_eef_spinbox(Eigen::VectorXd)\0"
};

void cat_manipulator_ik_gui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_getJointState_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_getEEF_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_getTagPose_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_setJointState_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_setEEF_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_publishJointState_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_move_gotoRest_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_move_gotoPos1_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_move_gotoPos2_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_move_gotoPos3_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->on_move_drop_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->on_move_pickX_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 13: _t->on_move_pickY_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->on_move_pickYback_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->on_move_gobackdroprobot_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_move_gobackdroptable_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->on_closegripper_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_opengripper_button_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 19: _t->updateEEFinWindow(); break;
        case 20: _t->updateJointStatesinWindow(); break;
        case 21: _t->sendJoints((*reinterpret_cast< Eigen::VectorXd(*)>(_a[1]))); break;
        case 22: _t->updateLoggingView(); break;
        case 23: _t->update_joint_states_spinbox((*reinterpret_cast< Eigen::VectorXd(*)>(_a[1]))); break;
        case 24: _t->update_eef_spinbox((*reinterpret_cast< Eigen::VectorXd(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cat_manipulator_ik_gui::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cat_manipulator_ik_gui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_cat_manipulator_ik_gui__MainWindow,
      qt_meta_data_cat_manipulator_ik_gui__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cat_manipulator_ik_gui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cat_manipulator_ik_gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cat_manipulator_ik_gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cat_manipulator_ik_gui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int cat_manipulator_ik_gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
