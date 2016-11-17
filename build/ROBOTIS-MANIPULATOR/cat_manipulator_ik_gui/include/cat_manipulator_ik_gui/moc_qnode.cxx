/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-MANIPULATOR/cat_manipulator_ik_gui/include/cat_manipulator_ik_gui/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cat_manipulator_ik_gui__QNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      31,   30,   30,   30, 0x05,
      48,   30,   30,   30, 0x05,
      79,   62,   30,   30, 0x05,
     124,  116,   30,   30, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_cat_manipulator_ik_gui__QNode[] = {
    "cat_manipulator_ik_gui::QNode\0\0"
    "loggingUpdated()\0rosShutdown()\0"
    "cur_joint_states\0update_joint_states(Eigen::VectorXd)\0"
    "cur_eef\0update_eef(Eigen::VectorXd)\0"
};

void cat_manipulator_ik_gui::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->rosShutdown(); break;
        case 2: _t->update_joint_states((*reinterpret_cast< Eigen::VectorXd(*)>(_a[1]))); break;
        case 3: _t->update_eef((*reinterpret_cast< Eigen::VectorXd(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData cat_manipulator_ik_gui::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject cat_manipulator_ik_gui::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_cat_manipulator_ik_gui__QNode,
      qt_meta_data_cat_manipulator_ik_gui__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cat_manipulator_ik_gui::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cat_manipulator_ik_gui::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cat_manipulator_ik_gui::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cat_manipulator_ik_gui__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int cat_manipulator_ik_gui::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void cat_manipulator_ik_gui::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void cat_manipulator_ik_gui::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void cat_manipulator_ik_gui::QNode::update_joint_states(Eigen::VectorXd _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void cat_manipulator_ik_gui::QNode::update_eef(Eigen::VectorXd _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
