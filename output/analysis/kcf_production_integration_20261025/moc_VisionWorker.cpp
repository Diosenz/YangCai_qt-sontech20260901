#include "RuntimeWorkerAdapters.h"
/****************************************************************************
** Meta object code from reading C++ file 'VisionWorkerMoc.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'VisionWorkerMoc.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t {};
} // unnamed namespace

template <> constexpr inline auto i2r_rlc::ui::CVisionTrackingWorkerAdapter::qt_create_metaobjectdata<qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "i2r_rlc::ui::CVisionTrackingWorkerAdapter",
        "trackingResultReady",
        "",
        "RuntimeTrackingResult",
        "result",
        "statusChanged",
        "message",
        "configure",
        "RuntimeTrackingOptions",
        "options",
        "vision::WeldProjectionContext",
        "projectionContext",
        "updateOnlineFusionOptions",
        "vision::KcfFusionOptions",
        "updateRobotState",
        "robot_comm::RobotState",
        "state",
        "updateDigitalTwinVisionObservation",
        "digital_twin::DigitalTwinVisionLaserObservation",
        "observation",
        "startTracking",
        "stopTracking",
        "submitFrame",
        "camera::CameraFrame",
        "frame",
        "submitFrameWithRobotState",
        "localizationSegmentIndex",
        "submitFrameWithRobotStateAndCorrelation",
        "RuntimeVisionCorrelation",
        "correlation",
        "submitFrameWithRobotStateAndObservation",
        "submitFrameWithRobotStateObservationAndCorrelation"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'trackingResultReady'
        QtMocHelpers::SignalData<void(const RuntimeTrackingResult &)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 },
        }}),
        // Signal 'statusChanged'
        QtMocHelpers::SignalData<void(const QString &)>(5, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 6 },
        }}),
        // Slot 'configure'
        QtMocHelpers::SlotData<void(const RuntimeTrackingOptions &, const vision::WeldProjectionContext &)>(7, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 8, 9 }, { 0x80000000 | 10, 11 },
        }}),
        // Slot 'updateOnlineFusionOptions'
        QtMocHelpers::SlotData<void(const vision::KcfFusionOptions &)>(12, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 13, 9 },
        }}),
        // Slot 'updateRobotState'
        QtMocHelpers::SlotData<void(const robot_comm::RobotState &)>(14, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 15, 16 },
        }}),
        // Slot 'updateDigitalTwinVisionObservation'
        QtMocHelpers::SlotData<void(const digital_twin::DigitalTwinVisionLaserObservation &)>(17, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 18, 19 },
        }}),
        // Slot 'startTracking'
        QtMocHelpers::SlotData<void()>(20, 2, QMC::AccessPublic, QMetaType::Void),
        // Slot 'stopTracking'
        QtMocHelpers::SlotData<void()>(21, 2, QMC::AccessPublic, QMetaType::Void),
        // Slot 'submitFrame'
        QtMocHelpers::SlotData<void(const camera::CameraFrame &)>(22, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 23, 24 },
        }}),
        // Slot 'submitFrameWithRobotState'
        QtMocHelpers::SlotData<void(const camera::CameraFrame &, const robot_comm::RobotState &, int)>(25, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 23, 24 }, { 0x80000000 | 15, 16 }, { QMetaType::Int, 26 },
        }}),
        // Slot 'submitFrameWithRobotStateAndCorrelation'
        QtMocHelpers::SlotData<void(const camera::CameraFrame &, const robot_comm::RobotState &, int, const RuntimeVisionCorrelation &)>(27, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 23, 24 }, { 0x80000000 | 15, 16 }, { QMetaType::Int, 26 }, { 0x80000000 | 28, 29 },
        }}),
        // Slot 'submitFrameWithRobotStateAndObservation'
        QtMocHelpers::SlotData<void(const camera::CameraFrame &, const robot_comm::RobotState &, int, const digital_twin::DigitalTwinVisionLaserObservation &)>(30, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 23, 24 }, { 0x80000000 | 15, 16 }, { QMetaType::Int, 26 }, { 0x80000000 | 18, 19 },
        }}),
        // Slot 'submitFrameWithRobotStateObservationAndCorrelation'
        QtMocHelpers::SlotData<void(const camera::CameraFrame &, const robot_comm::RobotState &, int, const digital_twin::DigitalTwinVisionLaserObservation &, const RuntimeVisionCorrelation &)>(31, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 23, 24 }, { 0x80000000 | 15, 16 }, { QMetaType::Int, 26 }, { 0x80000000 | 18, 19 },
            { 0x80000000 | 28, 29 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<CVisionTrackingWorkerAdapter, qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject i2r_rlc::ui::CVisionTrackingWorkerAdapter::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>.metaTypes,
    nullptr
} };

void i2r_rlc::ui::CVisionTrackingWorkerAdapter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<CVisionTrackingWorkerAdapter *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->trackingResultReady((*reinterpret_cast< std::add_pointer_t<RuntimeTrackingResult>>(_a[1]))); break;
        case 1: _t->statusChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->configure((*reinterpret_cast< std::add_pointer_t<RuntimeTrackingOptions>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<vision::WeldProjectionContext>>(_a[2]))); break;
        case 3: _t->updateOnlineFusionOptions((*reinterpret_cast< std::add_pointer_t<vision::KcfFusionOptions>>(_a[1]))); break;
        case 4: _t->updateRobotState((*reinterpret_cast< std::add_pointer_t<robot_comm::RobotState>>(_a[1]))); break;
        case 5: _t->updateDigitalTwinVisionObservation((*reinterpret_cast< std::add_pointer_t<digital_twin::DigitalTwinVisionLaserObservation>>(_a[1]))); break;
        case 6: _t->startTracking(); break;
        case 7: _t->stopTracking(); break;
        case 8: _t->submitFrame((*reinterpret_cast< std::add_pointer_t<camera::CameraFrame>>(_a[1]))); break;
        case 9: _t->submitFrameWithRobotState((*reinterpret_cast< std::add_pointer_t<camera::CameraFrame>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<robot_comm::RobotState>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3]))); break;
        case 10: _t->submitFrameWithRobotStateAndCorrelation((*reinterpret_cast< std::add_pointer_t<camera::CameraFrame>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<robot_comm::RobotState>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<RuntimeVisionCorrelation>>(_a[4]))); break;
        case 11: _t->submitFrameWithRobotStateAndObservation((*reinterpret_cast< std::add_pointer_t<camera::CameraFrame>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<robot_comm::RobotState>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<digital_twin::DigitalTwinVisionLaserObservation>>(_a[4]))); break;
        case 12: _t->submitFrameWithRobotStateObservationAndCorrelation((*reinterpret_cast< std::add_pointer_t<camera::CameraFrame>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<robot_comm::RobotState>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<digital_twin::DigitalTwinVisionLaserObservation>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<RuntimeVisionCorrelation>>(_a[5]))); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (CVisionTrackingWorkerAdapter::*)(const RuntimeTrackingResult & )>(_a, &CVisionTrackingWorkerAdapter::trackingResultReady, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (CVisionTrackingWorkerAdapter::*)(const QString & )>(_a, &CVisionTrackingWorkerAdapter::statusChanged, 1))
            return;
    }
}

const QMetaObject *i2r_rlc::ui::CVisionTrackingWorkerAdapter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *i2r_rlc::ui::CVisionTrackingWorkerAdapter::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN7i2r_rlc2ui28CVisionTrackingWorkerAdapterE_t>.strings))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int i2r_rlc::ui::CVisionTrackingWorkerAdapter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void i2r_rlc::ui::CVisionTrackingWorkerAdapter::trackingResultReady(const RuntimeTrackingResult & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 0, nullptr, _t1);
}

// SIGNAL 1
void i2r_rlc::ui::CVisionTrackingWorkerAdapter::statusChanged(const QString & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 1, nullptr, _t1);
}
QT_WARNING_POP
