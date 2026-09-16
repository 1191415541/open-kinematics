#pragma once

/// The joint-type registry: one descriptor per `AxleConstraintType` (MB_CONSTRAINT).
///
/// Every constraint operation used to be a hand-written dispatcher over the ten
/// joint types -- `constraint_rows`, `constraint_residual`, `constraint_jacobian`,
/// `constraint_jacobian_directional` and `constraint_residual_maxima` each spelled
/// the type list out again, and a type missing from one of them contributed
/// silently nothing (`MODULES.md` section 5.1).  The descriptor table is now the
/// only place the types are enumerated: adding a joint type means adding one row
/// plus its handlers, and all five call sites are untouched.
///
/// The row count and the residual classification are *not* duplicated here.  They
/// live in `mb_constraint/types.hpp`, which `mb_model` also reads, so the two
/// answers cannot drift apart.
///
/// Each handler is written in terms of a writer that carries the per-constraint
/// geometry the driver already computed and the two index-preserving row helpers.
/// That keeps a handler a straight transcription of the branch it replaces: no
/// handler re-derives the body arms or the offset, so the split cannot change what
/// is accumulated into which row.

#include <cstddef>
#include <vector>

#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_base/vector.hpp"
#include "mb_constraint/types.hpp"
#include "mb_model/types.hpp"

namespace axle_kernel {

/// The per-constraint state a residual handler writes into.
///
/// `row` is the first row the type owns, i.e. it already skips the shared
/// translation block when the type has one.
struct JointResidualContext {
    const Constraint& constraint;
    const State& state;
    const SampleInput* input;
    std::vector<double>& out;
    int row{0};
    Vec3 dp{};
};

/// The frame perpendicular to a joint axis, with the derivatives the Jacobian
/// rows need.  The revolute, prismatic and cylindrical joints all build their
/// rows on it, and they must build it the same way, so it is derived once.
struct AxisFrame {
    Vec3 aa{};
    Vec3 e1{};
    Vec3 e2{};
    Mat3 d_aa{};
    Mat3 d_e1{};
    Mat3 d_e2{};
};

/// The directional counterpart of `AxisFrame`.
struct DirectionalAxisFrame {
    DVec3 aa{};
    DVec3 e1{};
    DVec3 e2{};
    DMat3 d_aa{};
    DMat3 d_e1{};
    DMat3 d_e2{};
};

/// The row writer a scalar-Jacobian handler fills.
///
/// The geometry is the driver's, computed once per constraint before the handler
/// is called; the handlers only choose rows and coefficients.
struct ScalarJacobianWriter {
    const Model& model;
    const State& state;
    const Constraint& constraint;
    std::vector<double>& jacobian;
    /// First row this type owns -- after the shared translation block, if any.
    int row{0};
    Mat3 ra{};
    Vec3 arm_a{};
    Vec3 arm_b{};
    Vec3 dp{};

    /// Add one 6-vector into the block of `body`, doing nothing for a body that is
    /// not a free coordinate.
    void add_row(
        int row, int body, const Vec3& translation, const Vec3& rotation
    ) const;

    /// Add three rows at once, one per matrix row.
    void add_block(
        int row0, int body, const Mat3& translation, const Mat3& rotation
    ) const;

    /// The three rows of the relative rotation `q_a^-1 q_b`, in the same
    /// derivation the fixed joint and the couplers use.
    void add_relative_rotation(int row0) const;

    /// One row of the relative rotation projected on a body-a axis.
    void add_relative_rotation_row(int row0, const Vec3& axis_local) const;

    /// The perpendicular axis frame this constraint's axis implies.
    AxisFrame axis_frame() const;
};

/// The row writer a directional-Jacobian handler fills.
///
/// `smooth` is the driver's flag: a handler clears it where the derivative it
/// evaluated is not defined (the perpendicular reference degenerating, for
/// instance), exactly as the inline branches did.
struct DirectionalJacobianWriter {
    const Model& model;
    const State& state;
    const Constraint& constraint;
    const DirectionalState& direction;
    std::vector<double>& derivative;
    std::vector<std::size_t>* touched_indices;
    int ndof{0};
    /// First row this type owns -- after the shared translation block, if any.
    int row{0};
    DQuat qa{};
    DQuat qb{};
    DMat3 ra{};
    DVec3 arm_a{};
    DVec3 arm_b{};
    DVec3 dp{};
    bool& smooth;

    void add_row(
        int row, int body, const DVec3& translation, const DVec3& rotation
    ) const;
    void add_block(
        int row0, int body, const DMat3& translation, const DMat3& rotation
    ) const;
    void add_relative_rotation(int row0) const;
    void add_relative_rotation_row(int row0, const Vec3& axis_local) const;

    DirectionalAxisFrame axis_frame() const;
};

/// One joint type: its rows, whether it writes the shared point-coincidence
/// block, and one handler per operation.  A null handler means the type
/// contributes no row beyond the shared block, which is true of the spherical
/// joint in every operation.
///
/// `info` points into `mb_constraint/types.hpp`'s table, so `info->rows` is also
/// what `constraint_rows` reports.
struct JointTypeDescriptor {
    const JointTypeInfo* info{nullptr};
    bool translation_block{false};
    void (*write_residual)(const JointResidualContext&){nullptr};
    void (*write_jacobian)(const ScalarJacobianWriter&){nullptr};
    void (*write_jacobian_directional)(const DirectionalJacobianWriter&){nullptr};
};

/// The descriptor of `type`, or null for an unregistered type.  The dispatchers
/// turn that null into the loud failure the kernel used to reach through its
/// final `else` (a NaN row) rather than into a silently empty row.
const JointTypeDescriptor* joint_type_descriptor(int type);

// --- residual handlers -------------------------------------------------

void joint_residual_revolute(const JointResidualContext& ctx);
void joint_residual_fixed(const JointResidualContext& ctx);
void joint_residual_prismatic(const JointResidualContext& ctx);
void joint_residual_universal(const JointResidualContext& ctx);
void joint_residual_cylindrical(const JointResidualContext& ctx);
void joint_residual_inplane(const JointResidualContext& ctx);
void joint_residual_convel(const JointResidualContext& ctx);
void joint_residual_driven_translation(const JointResidualContext& ctx);
void joint_residual_driven_rotation(const JointResidualContext& ctx);

// --- scalar Jacobian handlers ------------------------------------------

void joint_jacobian_revolute(const ScalarJacobianWriter& writer);
void joint_jacobian_fixed(const ScalarJacobianWriter& writer);
void joint_jacobian_prismatic(const ScalarJacobianWriter& writer);
void joint_jacobian_universal(const ScalarJacobianWriter& writer);
void joint_jacobian_cylindrical(const ScalarJacobianWriter& writer);
void joint_jacobian_inplane(const ScalarJacobianWriter& writer);
void joint_jacobian_convel(const ScalarJacobianWriter& writer);
void joint_jacobian_driven_translation(const ScalarJacobianWriter& writer);
void joint_jacobian_driven_rotation(const ScalarJacobianWriter& writer);

// --- directional Jacobian handlers -------------------------------------

void joint_jacobian_directional_revolute(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_fixed(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_prismatic(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_universal(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_cylindrical(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_inplane(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_convel(const DirectionalJacobianWriter& writer);
void joint_jacobian_directional_driven_translation(
    const DirectionalJacobianWriter& writer
);
void joint_jacobian_directional_driven_rotation(
    const DirectionalJacobianWriter& writer
);

} // namespace axle_kernel
