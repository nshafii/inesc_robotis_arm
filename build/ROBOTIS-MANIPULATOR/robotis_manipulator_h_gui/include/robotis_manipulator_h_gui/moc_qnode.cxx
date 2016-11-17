/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/ROBOTIS-MANIPULATOR/robotis_manipulator_h_gui/include/robotis_manipulator_h_gui/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_robotis_manipulator_h_gui__QNode[] = {

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
      34,   33,   33,   33, 0x05,
      51,   33,   33,   33, 0x05,
      71,   65,   33,   33, 0x05,
     117,  109,   33,   33, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_robotis_manipulator_h_gui__QNode[] = {
    "robotis_manipulator_h_gui::QNode\0\0"
    "loggingUpdated()\0rosShutdown()\0x,y,z\0"
    "update_curr_pos(double,double,double)\0"
    "x,y,z,w\0update_curr_ori(double,double,double,double)\0"
};

void robotis_manipulator_h_gui::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->loggingUpdated(); break;
        case 1: _t->rosShutdown(); break;
        case 2: _t->update_curr_pos((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 3: _t->update_curr_ori((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData robotis_manipulator_h_gui::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject robotis_manipulator_h_gui::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_robotis_manipulator_h_gui__QNode,
      qt_meta_data_robotis_manipulator_h_gui__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &robotis_manipulator_h_gui::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *robotis_manipulator_h_gui::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *robotis_manipulator_h_gui::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_robotis_manipulator_h_gui__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int robotis_manipulator_h_gui::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void robotis_manipulator_h_gui::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void robotis_manipulator_h_gui::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void robotis_manipulator_h_gui::QNode::update_curr_pos(double _t1, double _t2, double _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void robotis_manipulator_h_gui::QNode::update_curr_ori(double _t1, double _t2, double _t3, double _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
