#pragma once

/// The joint types' row counts and residual classification (MB_CONSTRAINT).
///
/// This is the single source of the two facts every constraint dispatcher used to
/// spell out again for each of the ten types: how many rows a type contributes,
/// and which of those rows are positional rather than angular.  It has no C++
/// dependencies beyond the enumerations because `mb_model` asks for a row count
/// while it lays out `Model::rows`, and `mb_constraint` owns the answer
/// (`MODULES.md` section 2.4).  Keeping the data here, rather than in
/// `mb_constraint/registry.hpp`, is what stops `model -> constraint` from
/// becoming a *type* dependency: only this header crosses that edge, and it
/// needs nothing but `mb_model/enums.hpp`.

#include "mb_model/enums.hpp"

namespace axle_kernel {

/// One run of consecutive residual rows of a single joint type.
///
/// `constraint_residual_maxima` compares positional violations against a length
/// tolerance and angular ones against an angle tolerance, so a type's rows are
/// described as ordered runs rather than as two totals: the cylindrical and
/// prismatic joints interleave the kinds (offset rows first, then parallelism
/// rows), and a total per kind would lose that.
struct JointResidualClass {
    int count{0};
    bool angular{false};
};

/// The static, type-level facts about one joint type.
///
/// The number of runs never exceeds three, so the array is fixed-size and the
/// whole table stays a literal that can be read at compile time.
struct JointTypeInfo {
    int rows{-1};
    int class_count{0};
    JointResidualClass classes[3]{};
};

/// The number of registered joint types; every `AxleConstraintType` is below it.
constexpr int kJointTypeCount = AXLE_DRIVEN_ROTATION + 1;

/// The row layout of the ten joint types, plus one sentinel row at the end whose
/// `rows` is -1.  `joint_type_info` returns the sentinel for an unregistered
/// type, which is how `constraint_rows` keeps reporting the -1 that
/// `build_model` rejects.
constexpr JointTypeInfo kJointTypeInfo[kJointTypeCount + 1] = {
    // AXLE_SPHERICAL: the coincident point only.
    {3, 1, {{3, false}, {0, false}, {0, false}}},
    // AXLE_REVOLUTE: the point, plus the two rows that keep the axes parallel.
    {5, 2, {{3, false}, {2, true}, {0, false}}},
    // AXLE_FIXED: the point, plus the full relative rotation.
    {6, 2, {{3, false}, {3, true}, {0, false}}},
    // AXLE_PRISMATIC: the perpendicular offset, the parallel axes, and the
    // relative spin about the axis.
    {5, 2, {{2, false}, {3, true}, {0, false}}},
    // AXLE_UNIVERSAL: the point, plus one cross-axis orthogonality row.
    {4, 2, {{3, false}, {1, true}, {0, false}}},
    // AXLE_CYLINDRICAL: the perpendicular offset, then the parallel axes.
    {4, 2, {{2, false}, {2, true}, {0, false}}},
    // AXLE_INPLANE: B's point against A's plane normal.
    {1, 1, {{1, false}, {0, false}, {0, false}}},
    // AXLE_CONVEL: the point, plus the constant-velocity coupling row.
    {4, 2, {{3, false}, {1, true}, {0, false}}},
    // AXLE_DRIVEN_TRANSLATION: the prescribed separation along the axis.
    {1, 1, {{1, false}, {0, false}, {0, false}}},
    // AXLE_DRIVEN_ROTATION: the prescribed rotation about the axis.
    {1, 1, {{1, true}, {0, false}, {0, false}}},
    // Not a joint type: `constraint_rows` reports -1 and `build_model` rejects.
    {-1, 0, {{0, false}, {0, false}, {0, false}}},
};

/// The description of `type`; an unregistered type gets the sentinel.
constexpr const JointTypeInfo& joint_type_info(int type) {
    return type >= 0 && type < kJointTypeCount
        ? kJointTypeInfo[type]
        : kJointTypeInfo[kJointTypeCount];
}

/// Whether every entry above accounts for exactly its own rows.  A joint type
/// whose runs do not add up would silently drop rows from the residual maxima, so
/// the table is checked where it is written rather than at runtime.
constexpr bool joint_type_info_consistent() {
    for (int type = 0; type < kJointTypeCount; ++type) {
        const JointTypeInfo& info = kJointTypeInfo[type];
        if (info.rows < 0 || info.class_count < 1 || info.class_count > 3) {
            return false;
        }
        int total = 0;
        for (int run = 0; run < info.class_count; ++run) {
            if (info.classes[run].count < 1) return false;
            total += info.classes[run].count;
        }
        if (total != info.rows) return false;
    }
    return true;
}
static_assert(
    joint_type_info_consistent(),
    "each joint type's residual runs must account for exactly its rows"
);

/// The classification result the statics layer compares against its tolerances.
///
/// It is the consumer side of the table above: the initial-state gate for a
/// `provided_consistent_state` run judges positional violations against a length
/// tolerance and angular ones against an angle tolerance, so the two maxima are
/// reported separately.  It lives with the classification rather than with the
/// statics unit that fills it.
struct ConstraintResidualMaxima {
    double position{0.0};
    double angle{0.0};
};

} // namespace axle_kernel
