/****************************************************************************
** Meta object code from reading C++ file 'whirlybird_panel_pwm.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../../fsf/jaredeo/ME431_Controls/whirlybird_ws/src/whirlybird_description/src/whirlybird_panel_pwm.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'whirlybird_panel_pwm.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM_t {
    QByteArrayData data[10];
    char stringdata0[121];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM_t qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM = {
    {
QT_MOC_LITERAL(0, 0, 42), // "whirlybird_description::Whirl..."
QT_MOC_LITERAL(1, 43, 6), // "setPWM"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 3), // "pwm"
QT_MOC_LITERAL(4, 55, 11), // "setPWMTopic"
QT_MOC_LITERAL(5, 67, 5), // "topic"
QT_MOC_LITERAL(6, 73, 10), // "setEnabled"
QT_MOC_LITERAL(7, 84, 7), // "enabled"
QT_MOC_LITERAL(8, 92, 13), // "sendSetpoints"
QT_MOC_LITERAL(9, 106, 14) // "updatePWMTopic"

    },
    "whirlybird_description::WhirlybirdPanelPWM\0"
    "setPWM\0\0pwm\0setPWMTopic\0topic\0setEnabled\0"
    "enabled\0sendSetpoints\0updatePWMTopic"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_whirlybird_description__WhirlybirdPanelPWM[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x0a /* Public */,
       4,    1,   42,    2, 0x0a /* Public */,
       6,    1,   45,    2, 0x0a /* Public */,
       8,    0,   48,    2, 0x09 /* Protected */,
       9,    0,   49,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void whirlybird_description::WhirlybirdPanelPWM::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        WhirlybirdPanelPWM *_t = static_cast<WhirlybirdPanelPWM *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setPWM((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->setPWMTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->setEnabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->sendSetpoints(); break;
        case 4: _t->updatePWMTopic(); break;
        default: ;
        }
    }
}

const QMetaObject whirlybird_description::WhirlybirdPanelPWM::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM.data,
      qt_meta_data_whirlybird_description__WhirlybirdPanelPWM,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *whirlybird_description::WhirlybirdPanelPWM::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *whirlybird_description::WhirlybirdPanelPWM::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_whirlybird_description__WhirlybirdPanelPWM.stringdata0))
        return static_cast<void*>(const_cast< WhirlybirdPanelPWM*>(this));
    return rviz::Panel::qt_metacast(_clname);
}

int whirlybird_description::WhirlybirdPanelPWM::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
