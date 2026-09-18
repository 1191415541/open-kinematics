// The joint-type registry (MODULES.md section 5.1).
//
// This unit owns the one table that says what each joint type is and how to
// evaluate it: the row writer helpers every handler shares, and the descriptor
// table that binds a type to its four operations.  The handlers themselves stay
// with the math they transcribe -- the residual and scalar Jacobian handlers in
// `kernel_model_constraint.cpp`, the directional ones in
// `kernel_directional_constraint.cpp` -- so this file stays declarative.

#include "mb_constraint/functions.hpp"

#include "mb_constraint/registry.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_base/functions.hpp"

namespace axle_kernel {

// --- the row writers ---------------------------------------------------

void ScalarJacobianWriter::add_row(
    int row, int body, const Vec3& translation, const Vec3& rotation
) const {
    const int fi = model.body_to_free[body];
    if (fi < 0) return;
    double* out = &jacobian[static_cast<std::size_t>(row)*model.ndof + 6*fi];
    out[0] += translation.x; out[1] += translation.y; out[2] += translation.z;
    out[3] += rotation.x;    out[4] += rotation.y;    out[5] += rotation.z;
}

void ScalarJacobianWriter::add_block(
    int row0, int body, const Mat3& translation, const Mat3& rotation
) const {
    for (int i = 0; i < 3; ++i) {
        add_row(
            row0+i, body,
            {translation.a[i][0], translation.a[i][1], translation.a[i][2]},
            {rotation.a[i][0], rotation.a[i][1], rotation.a[i][2]}
        );
    }
}

void ScalarJacobianWriter::add_relative_rotation(int row0) const {
    const Vec3 phi = qlog(
        qmul(qconj(state.q[constraint.a]), state.q[constraint.b])
    );
    const Mat3 map = log_left_jacobian_inverse(phi) * transpose(ra);
    const Mat3 zero{};
    add_block(row0, constraint.a, zero, map*(-1.0));
    add_block(row0, constraint.b, zero, map);
}

void ScalarJacobianWriter::add_relative_rotation_row(
    int row0, const Vec3& axis_local
) const {
    const Vec3 phi = qlog(
        qmul(qconj(state.q[constraint.a]), state.q[constraint.b])
    );
    const Mat3 map = log_left_jacobian_inverse(phi) * transpose(ra);
    const Vec3 row = row_times(normalized(axis_local), map);
    add_row(row0, constraint.a, {}, row*(-1.0));
    add_row(row0, constraint.b, {}, row);
}

AxisFrame ScalarJacobianWriter::axis_frame() const {
    const Mat3 eye = identity3();
    AxisFrame frame;
    frame.aa = normalized(rotate(state.q[constraint.a], constraint.axis_a));
    const Vec3 g = perpendicular_reference(frame.aa);
    const Vec3 u = g - frame.aa*dot(g, frame.aa);
    const double u_norm = norm(u);
    frame.e1 = u * (1.0/u_norm);
    frame.e2 = cross(frame.aa, frame.e1);
    // Rotation preserves length, so normalizing commutes with it and
    // d(aa)/d(delta theta_a) = -[aa]x.
    frame.d_aa = skew(frame.aa)*(-1.0);
    const Mat3 d_u = (eye*dot(g, frame.aa) + outer(frame.aa, g))*(-1.0);
    frame.d_e1 =
        ((eye - outer(frame.e1, frame.e1))*(1.0/u_norm)) * d_u * frame.d_aa;
    frame.d_e2 = skew(frame.e1)*(-1.0)*frame.d_aa + skew(frame.aa)*frame.d_e1;
    return frame;
}

void DirectionalJacobianWriter::add_row(
    int row, int body, const DVec3& translation, const DVec3& rotation
) const {
    const int fi = model.body_to_free[static_cast<std::size_t>(body)];
    if (fi < 0) return;
    double* out = &derivative[static_cast<std::size_t>(row*ndof+6*fi)];
    if (touched_indices != nullptr) {
        for (int k = 0; k < 6; ++k) {
            touched_indices->push_back(
                static_cast<std::size_t>(row*ndof+6*fi+k)
            );
        }
    }
    out[0] += translation.x.derivative;
    out[1] += translation.y.derivative;
    out[2] += translation.z.derivative;
    out[3] += rotation.x.derivative;
    out[4] += rotation.y.derivative;
    out[5] += rotation.z.derivative;
}

void DirectionalJacobianWriter::add_block(
    int row0, int body, const DMat3& translation, const DMat3& rotation
) const {
    for (int i = 0; i < 3; ++i) {
        add_row(
            row0+i, body,
            {translation.a[i][0], translation.a[i][1], translation.a[i][2]},
            {rotation.a[i][0], rotation.a[i][1], rotation.a[i][2]}
        );
    }
}

void DirectionalJacobianWriter::add_relative_rotation(int row0) const {
    const DVec3 phi = d_qlog(d_qmul(d_qconj(qa), qb), smooth);
    const DMat3 map =
        d_log_left_jacobian_inverse(phi, smooth)*d_transpose(ra);
    const DMat3 zero{};
    add_block(row0, constraint.a, zero, map*(-1.0));
    add_block(row0, constraint.b, zero, map);
}

void DirectionalJacobianWriter::add_relative_rotation_row(
    int row0, const Vec3& axis_local
) const {
    const DVec3 phi = d_qlog(d_qmul(d_qconj(qa), qb), smooth);
    const DMat3 map =
        d_log_left_jacobian_inverse(phi, smooth)*d_transpose(ra);
    const DVec3 axis(axis_local.x, axis_local.y, axis_local.z);
    const DVec3 row = d_row_times(d_normalized(axis, smooth), map);
    add_row(row0, constraint.a, {}, -row);
    add_row(row0, constraint.b, {}, row);
}

DirectionalAxisFrame DirectionalJacobianWriter::axis_frame() const {
    const DMat3 eye = d_identity3();
    DirectionalAxisFrame frame;
    frame.aa = d_normalized(
        d_rotate(
            state.q[constraint.a], constraint.axis_a,
            direction.dtheta[constraint.a]
        ),
        smooth
    );
    const Vec3 reference = perpendicular_reference(frame.aa.value());
    if (std::abs(std::abs(frame.aa.value().x)-0.8) <= 1e-12) {
        smooth = false;
    }
    const DVec3 g(reference.x, reference.y, reference.z);
    const DirectionalScalar g_dot_aa = d_dot(g, frame.aa);
    const DVec3 u = g-frame.aa*g_dot_aa;
    const DirectionalScalar u_norm = d_norm(u);
    if (u_norm.value <= kEps) smooth = false;
    frame.e1 = u/u_norm;
    frame.e2 = d_cross(frame.aa, frame.e1);
    frame.d_aa = d_skew(frame.aa)*(-1.0);
    const DMat3 d_u = (eye*g_dot_aa+d_outer(frame.aa, g))*(-1.0);
    frame.d_e1 =
        ((eye-d_outer(frame.e1, frame.e1))*(1.0/u_norm))*d_u*frame.d_aa;
    frame.d_e2 = d_skew(frame.e1)*(-1.0)*frame.d_aa+d_skew(frame.aa)*frame.d_e1;
    return frame;
}

// --- the table ---------------------------------------------------------

namespace {
const JointTypeDescriptor kDescriptors[kJointTypeCount] = {
    // The spherical joint writes the shared point-coincidence block and nothing
    // else, in every one of the five operations: its handlers are null.
    {&kJointTypeInfo[AXLE_SPHERICAL], true, nullptr, nullptr, nullptr},
    {&kJointTypeInfo[AXLE_REVOLUTE], true,
     joint_residual_revolute, joint_jacobian_revolute,
     joint_jacobian_directional_revolute},
    {&kJointTypeInfo[AXLE_FIXED], true,
     joint_residual_fixed, joint_jacobian_fixed,
     joint_jacobian_directional_fixed},
    {&kJointTypeInfo[AXLE_PRISMATIC], false,
     joint_residual_prismatic, joint_jacobian_prismatic,
     joint_jacobian_directional_prismatic},
    {&kJointTypeInfo[AXLE_UNIVERSAL], true,
     joint_residual_universal, joint_jacobian_universal,
     joint_jacobian_directional_universal},
    {&kJointTypeInfo[AXLE_CYLINDRICAL], false,
     joint_residual_cylindrical, joint_jacobian_cylindrical,
     joint_jacobian_directional_cylindrical},
    {&kJointTypeInfo[AXLE_INPLANE], false,
     joint_residual_inplane, joint_jacobian_inplane,
     joint_jacobian_directional_inplane},
    {&kJointTypeInfo[AXLE_CONVEL], true,
     joint_residual_convel, joint_jacobian_convel,
     joint_jacobian_directional_convel},
    {&kJointTypeInfo[AXLE_DRIVEN_TRANSLATION], false,
     joint_residual_driven_translation, joint_jacobian_driven_translation,
     joint_jacobian_directional_driven_translation},
    {&kJointTypeInfo[AXLE_DRIVEN_ROTATION], false,
     joint_residual_driven_rotation, joint_jacobian_driven_rotation,
     joint_jacobian_directional_driven_rotation},
};
static_assert(
    sizeof(kDescriptors)/sizeof(kDescriptors[0]) ==
        static_cast<std::size_t>(kJointTypeCount),
    "every joint type needs a descriptor row"
);
} // namespace

const JointTypeDescriptor* joint_type_descriptor(int type) {
    if (type < 0 || type >= kJointTypeCount) return nullptr;
    // A row that was never filled in is treated as unregistered, so a new
    // enumerator fails the way an unknown type does instead of dereferencing a
    // null `info`.
    if (kDescriptors[type].info == nullptr) return nullptr;
    return &kDescriptors[type];
}

} // namespace axle_kernel
