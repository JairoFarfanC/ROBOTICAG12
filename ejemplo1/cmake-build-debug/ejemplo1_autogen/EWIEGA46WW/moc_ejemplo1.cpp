/****************************************************************************
** Meta object code from reading C++ file 'ejemplo1.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ejemplo1.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ejemplo1.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_ejemplo1_t {
    uint offsetsAndSizes[16];
    char stringdata0[9];
    char stringdata1[9];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[5];
    char stringdata5[8];
    char stringdata6[6];
    char stringdata7[9];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_ejemplo1_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_ejemplo1_t qt_meta_stringdata_ejemplo1 = {
    {
        QT_MOC_LITERAL(0, 8),  // "ejemplo1"
        QT_MOC_LITERAL(9, 8),  // "doButton"
        QT_MOC_LITERAL(18, 0),  // ""
        QT_MOC_LITERAL(19, 7),  // "doCount"
        QT_MOC_LITERAL(27, 4),  // "step"
        QT_MOC_LITERAL(32, 7),  // "doSlide"
        QT_MOC_LITERAL(40, 5),  // "reset"
        QT_MOC_LITERAL(46, 8)   // "doRandom"
    },
    "ejemplo1",
    "doButton",
    "",
    "doCount",
    "step",
    "doSlide",
    "reset",
    "doRandom"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_ejemplo1[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   44,    2, 0x0a,    1 /* Public */,
       3,    1,   45,    2, 0x0a,    2 /* Public */,
       5,    0,   48,    2, 0x0a,    4 /* Public */,
       6,    0,   49,    2, 0x0a,    5 /* Public */,
       7,    0,   50,    2, 0x0a,    6 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Int,

       0        // eod
};

Q_CONSTINIT const QMetaObject ejemplo1::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ejemplo1.offsetsAndSizes,
    qt_meta_data_ejemplo1,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_ejemplo1_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ejemplo1, std::true_type>,
        // method 'doButton'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'doCount'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'doSlide'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'reset'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'doRandom'
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void ejemplo1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ejemplo1 *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->doButton(); break;
        case 1: _t->doCount((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 2: _t->doSlide(); break;
        case 3: _t->reset(); break;
        case 4: { int _r = _t->doRandom();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject *ejemplo1::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ejemplo1::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ejemplo1.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Ui_Counter"))
        return static_cast< Ui_Counter*>(this);
    return QWidget::qt_metacast(_clname);
}

int ejemplo1::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
