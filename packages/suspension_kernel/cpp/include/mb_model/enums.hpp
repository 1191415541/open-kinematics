#pragma once

// The kernel's enumerations, in a header with no dependencies of its own.
//
// These used to be defined by the public C ABI header, which meant every layer
// that compared a `VehicleTireModelKind` or asked for `constraint_rows` depended
// on the top-level ABI header -- and the ABI header is the one thing that has to
// stay stable, so nothing should be dragging it into the model and tire layers.
// Moving them here makes the dependency `abi -> mb_model` instead of a cycle
// (epic section 8-D7).
//
// The values are ABI: a caller compiles against them, so an enumerator may be
// appended but never renumbered or removed.
//
// The header must stay free of C++ dependencies: it is included by the ABI header
// and by the model layer, in both directions.

#include <cstddef>

// The ABI version constants travel with these enumerations because the public ABI
// header used to define both, and its includers relied on getting both from it.
// Keeping the include here preserves that without making the model layer depend on
// the ABI header.
#include "abi/version.hpp"

extern "C" {

// Joint types.  The row count of each is in `constraint_rows`.
enum AxleConstraintType {
    AXLE_SPHERICAL = 0,
    AXLE_REVOLUTE = 1,
    AXLE_FIXED = 2,
    AXLE_PRISMATIC = 3,
    // Coincident points plus one orthogonality condition between the two
    // axes: four rows, two rotational degrees of freedom.
    AXLE_UNIVERSAL = 4,
    // Shared axis line, free to slide along and spin about it: four rows.
    AXLE_CYLINDRICAL = 5,
    // Point of body B constrained to the plane of body A: one row.
    AXLE_INPLANE = 6,
    // Adams CONVEL: coincident centers plus the constant-velocity relation.
    AXLE_CONVEL = 7,
    // A driven coordinate removes exactly the one degree of freedom it
    // prescribes.  Translation is a position along an axis, rotation an angle
    // about one; both take a target and a target rate per sample.
    AXLE_DRIVEN_TRANSLATION = 8,
    AXLE_DRIVEN_ROTATION = 9
};

// Steering actuator types for the vehicle surface.
enum VehicleSteeringActuatorType {
    // 平移输入的执行器；目标通过高刚度力近似。
    VEHICLE_STEERING_TRANSLATION = 0,
    VEHICLE_STEERING_ROTATION = 1,
    // 受约束的转角输入；目标进入 KKT 约束，不通过高刚度力近似。
    VEHICLE_STEERING_PRESCRIBED_ROTATION = 2,
    // 受约束的齿条平移输入；目标进入 KKT 约束。
    VEHICLE_STEERING_PRESCRIBED_TRANSLATION = 3
};

enum VehicleTireModelKind {
    VEHICLE_TIRE_NATIVE_BRUSH = 0,
    // 使用当前项目实现的 PAC2002 纯滑移/联合滑移与松弛项；
    // 为保持历史行为，零滑移处移除校准偏置。
    VEHICLE_TIRE_PAC2002_PURE_SLIP = 1,
    // Adams 内置 PAC2002 源模式。按照 PAC2002/Chrono 的 Fx0/Fy0
    // 定义保留 PH*/PV* 在零滑移处产生的源偏置。
    VEHICLE_TIRE_PAC2002_ADAMS_SOURCE = 2,
    // Adams FIALA 属性文件对应的瞬态刷模型。
    VEHICLE_TIRE_FIALA = 3
};

enum VehicleBushingRotationCoordinates {
    VEHICLE_BUSHING_ROTATION_VECTOR = 0,
    VEHICLE_BUSHING_CARDAN_XYZ = 1
};

// The PAC2002 parameter table's slot count.  It is a count, not a stride: the
// table is a fixed-size array on `Tire` and the ctypes mirror sizes itself from
// this same number, so the two cannot disagree.
enum { VEHICLE_PAC2002_PARAMETER_COUNT = 226 };

} // extern "C"
