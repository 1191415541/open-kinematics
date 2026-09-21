// The force-assembly primitives shared by the element laws and the tire
// laws, split out of `mb_force` at subtask 04 review: they are element-tier
// helpers (bushing deformation is part of the bushing law), so they live in
// `mb_element` and both the element and tire layers may call them.

#include "mb_element/functions.hpp"

namespace axle_kernel {

Vec3 add_force_on_body(
    std::vector<Vec3>& force, std::vector<Vec3>& torque,
    const Model& model, const State& state, int body, const Vec3& point_local,
    const Vec3& f_world) {
    if (body < 0 || model.bodies[body].fixed) return {};
    const Vec3 arm = rotate(state.q[body], point_local);
    force[body] += f_world;
    torque[body] += cross(arm, f_world);
    return f_world;
}

void add_torque_on_body(
    std::vector<Vec3>& torque, const Model& model, int body, const Vec3& tau_world
) {
    if (body < 0 || model.bodies[body].fixed) return;
    torque[body] += tau_world;
}

std::array<double, 6> mat6_mul(
    const std::array<double, 36>& matrix, const std::array<double, 6>& vector
) {
    std::array<double, 6> result{};
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            result[row] += matrix[static_cast<std::size_t>(row * 6 + col)] * vector[col];
        }
    }
    return result;
}

std::array<double, 6> bushing_deformation(
    const Bushing& bushing, const Model& model, const State& state,
    std::array<double, 6>& rate
) {
    const Vec3 pa = state_point(state, bushing.a, bushing.pa);
    const Vec3 pb = state_point(state, bushing.b, bushing.pb);
    const Vec3 va = state_point_velocity(state, bushing.a, bushing.pa);
    const Vec3 vb = state_point_velocity(state, bushing.b, bushing.pb);
    const Quat qfa = qmul(state.q[bushing.a], bushing.frame_a);
    const Quat qfb = qmul(state.q[bushing.b], bushing.frame_b);
    const Mat3 rfa = qmat(qfa);
    const Mat3 rt = transpose(rfa);
    const Vec3 rel = rt * (pb - pa);
    const Vec3 rel_v = rt * (vb - va);
    const Vec3 omega_a = rt * state.omega[bushing.a];
    const Vec3 omega_b = rt * state.omega[bushing.b];
    const Vec3 rel_rate = rel_v - cross(omega_a, rel);
    const Vec3 rel_omega = omega_b - omega_a;
    const Quat qrel = qmul(qconj(qfa), qfb);
    const Quat qdelta = qmul(qconj(bushing.reference), qrel);
    const Vec3 rotation = bushing.rotation_coordinates ==
            VEHICLE_BUSHING_CARDAN_XYZ
        ? cardan_xyz_from_rotation(qmat(qdelta))
        : qlog(qdelta);
    const Vec3 rotation_rate = bushing.rotation_coordinates ==
            VEHICLE_BUSHING_CARDAN_XYZ
        ? cardan_xyz_rate(rotation, rel_omega)
        : rel_omega;
    const Vec3 translation = rel - bushing.reference_translation;
    std::array<double, 6> deformation{
        translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z
    };
    rate = {
        rel_rate.x, rel_rate.y, rel_rate.z,
        rotation_rate.x, rotation_rate.y, rotation_rate.z
    };
    (void)model;
    return deformation;
}


} // namespace axle_kernel
