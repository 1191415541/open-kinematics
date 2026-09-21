// The model builder: turns an `AxleInput` into a solver-ready `Model`.
//
// This file began as the K2 `static_output` module, which held the statics, the
// output writers, the vehicle row registration and this builder together.  The
// first three were split off in K6 rounds 22-25, leaving only `build_model`; the
// name now says what the file actually contains.  The K2 banner's line range
// described the whole former module, so it was dropped rather than left to
// misdescribe a file that is now one function.

#include "mb_joint/functions.hpp"
#include "mb_assembly/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_config/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_model/functions.hpp"

namespace axle_kernel {



Model build_model(
    const AxleInput& in, std::string& error,
    const double* axis_a_secondary,
    const double* axis_b_secondary,
    const double* convel_angle_target,
    std::size_t coordinate_coupler_count,
    const int* coordinate_coupler_joint_a,
    const int* coordinate_coupler_coordinate_a,
    const double* coordinate_coupler_scale_a,
    const int* coordinate_coupler_joint_b,
    const int* coordinate_coupler_coordinate_b,
    const double* coordinate_coupler_scale_b) {
    Model m;
    if (!in.body_mass || !in.body_inertia_body_3x3 || !in.body_pose_position_quaternion ||
        in.body_count==0) { error="body arrays are missing"; return m; }
    m.bodies.resize(in.body_count);
    m.body_to_free.assign(in.body_count,-1);
    // Generic element surface: when the caller supplies element blocks they are
    // the model's elements and the per-family arrays below must stay empty.  The
    // lambda is also the guard at every family loop, so a caller that supplies
    // both forms is told rather than silently having one of them ignored.
    const auto element_blocks_supplied = [&in]() { return in.element_count > 0; };
    if (!read_element_blocks(
            in.elements, in.element_count, in.element_curves,
            in.topology_extension_count, in.topology_extensions, m, error)) {
        return m;
    }
    const auto only_when_no_blocks = [&](const char* family) {
        if (element_blocks_supplied()) {
            error = std::string(family) +
                " array is set while element blocks are also supplied";
            return false;
        }
        return true;
    };
    for (std::size_t i=0;i<in.body_count;++i) {
        Body& b=m.bodies[i];
        b.mass=in.body_mass[i];
        b.fixed=in.body_fixed && in.body_fixed[i]!=0;
        if (!(b.mass>0.0) && !b.fixed) { error="free body mass must be positive"; return m; }
        for(int r=0;r<3;++r)for(int c=0;c<3;++c)b.inertia_body.a[r][c]=in.body_inertia_body_3x3[(i*9)+r*3+c];
        if (!finite_symmetric(b.inertia_body) ||
            (!b.fixed && !symmetric_positive_definite(b.inertia_body))) {
            error="body inertia must be finite and symmetric; free-body inertia must be positive definite";
            return m;
        }
        const double* p=&in.body_pose_position_quaternion[i*7];
        const Quat body_quaternion{p[3],p[4],p[5],p[6]};
        if (!unit_quaternion(body_quaternion)) {
            error="body quaternion must be finite and unit length";
            return m;
        }
        b.r={p[0],p[1],p[2]}; b.q=qnormalize(body_quaternion);
        const double* vw=&in.body_velocity_omega[i*6];
        b.v={vw[0],vw[1],vw[2]}; b.omega={vw[3],vw[4],vw[5]};
        if (!std::isfinite(b.r.x) || !std::isfinite(b.r.y) || !std::isfinite(b.r.z) ||
            !std::isfinite(b.v.x) || !std::isfinite(b.v.y) || !std::isfinite(b.v.z) ||
            !std::isfinite(b.omega.x) || !std::isfinite(b.omega.y) || !std::isfinite(b.omega.z)) {
            error="body pose and velocity arrays must be finite";
            return m;
        }
    }
    for(std::size_t i=0;i<in.body_count;++i) if(!m.bodies[i].fixed){m.body_to_free[i]=static_cast<int>(m.free_body.size());m.free_body.push_back(static_cast<int>(i));}
    m.ndof=6*static_cast<int>(m.free_body.size());
    for(std::size_t i=0;i<in.constraint_count;++i){
        Constraint c;
        c.type=in.constraint_type[i]; c.a=in.constraint_body_a[i]; c.b=in.constraint_body_b[i];
        if(c.a<0||c.b<0||c.a>=static_cast<int>(in.body_count)||c.b>=static_cast<int>(in.body_count)){error="constraint body index out of range";return m;}
        c.pa={in.constraint_point_a[i*3],in.constraint_point_a[i*3+1],in.constraint_point_a[i*3+2]};
        c.pb={in.constraint_point_b[i*3],in.constraint_point_b[i*3+1],in.constraint_point_b[i*3+2]};
        c.axis_a={in.constraint_axis_a[i*3],in.constraint_axis_a[i*3+1],in.constraint_axis_a[i*3+2]};
        c.axis_b={in.constraint_axis_b[i*3],in.constraint_axis_b[i*3+1],in.constraint_axis_b[i*3+2]};
        if (axis_a_secondary != nullptr) {
            c.axis_a_secondary={
                axis_a_secondary[i*3], axis_a_secondary[i*3+1],
                axis_a_secondary[i*3+2]
            };
        }
        if (axis_b_secondary != nullptr) {
            c.axis_b_secondary={
                axis_b_secondary[i*3], axis_b_secondary[i*3+1],
                axis_b_secondary[i*3+2]
            };
        }
        if (convel_angle_target != nullptr) {
            c.convel_angle_target = convel_angle_target[i];
            if (!std::isfinite(c.convel_angle_target) ||
                std::abs(c.convel_angle_target) > 2.0) {
                error="constant-velocity angle target must be finite and in [-2,2]";
                return m;
            }
        }
        if(norm(c.axis_a)<kEps||norm(c.axis_b)<kEps){error="constraint axis must be nonzero";return m;}
        if (c.type == AXLE_CONVEL &&
            (norm(c.axis_a_secondary) < kEps || norm(c.axis_b_secondary) < kEps)) {
            error="constant-velocity secondary axis must be nonzero";
            return m;
        }
        c.row=m.rows; const int rows=constraint_rows(c.type); if(rows<0){error="unsupported constraint type";return m;}
        m.rows+=rows; m.constraints.push_back(c);
    }
    if (coordinate_coupler_count > 0 && (
        coordinate_coupler_joint_a == nullptr ||
        coordinate_coupler_coordinate_a == nullptr ||
        coordinate_coupler_scale_a == nullptr ||
        coordinate_coupler_joint_b == nullptr ||
        coordinate_coupler_coordinate_b == nullptr ||
        coordinate_coupler_scale_b == nullptr
    )) {
        error = "vehicle coordinate coupler arrays are missing";
        return m;
    }
    for (std::size_t i = 0; i < coordinate_coupler_count; ++i) {
        CoordinateCoupler coupler;
        coupler.joint_a = coordinate_coupler_joint_a[i];
        coupler.coordinate_a = coordinate_coupler_coordinate_a[i];
        coupler.scale_a = coordinate_coupler_scale_a[i];
        coupler.joint_b = coordinate_coupler_joint_b[i];
        coupler.coordinate_b = coordinate_coupler_coordinate_b[i];
        coupler.scale_b = coordinate_coupler_scale_b[i];
        if (coupler.joint_a < 0 ||
            coupler.joint_b < 0 ||
            coupler.joint_a >= static_cast<int>(m.constraints.size()) ||
            coupler.joint_b >= static_cast<int>(m.constraints.size()) ||
            coupler.joint_a == coupler.joint_b ||
            (coupler.coordinate_a != 0 && coupler.coordinate_a != 1) ||
            (coupler.coordinate_b != 0 && coupler.coordinate_b != 1) ||
            !std::isfinite(coupler.scale_a) ||
            !std::isfinite(coupler.scale_b) ||
            std::abs(coupler.scale_a) <= kEps ||
            std::abs(coupler.scale_b) <= kEps) {
            error = "invalid vehicle coordinate coupler";
            return m;
        }
        const Constraint& first = m.constraints[
            static_cast<std::size_t>(coupler.joint_a)
        ];
        const Constraint& second = m.constraints[
            static_cast<std::size_t>(coupler.joint_b)
        ];
        const auto coordinate_supported = [](const Constraint& joint,
                                             int coordinate) {
            if (coordinate == 0) {
                return joint.type == AXLE_REVOLUTE ||
                    joint.type == AXLE_CYLINDRICAL;
            }
            return joint.type == AXLE_PRISMATIC ||
                joint.type == AXLE_CYLINDRICAL;
        };
        if (!coordinate_supported(first, coupler.coordinate_a) ||
            !coordinate_supported(second, coupler.coordinate_b)) {
            error = "vehicle coordinate coupler uses an incompatible joint coordinate";
            return m;
        }
        const auto coordinate_reference = [&](const Constraint& joint,
                                              int coordinate,
                                              Quat& rotation) {
            if (coordinate == 0) {
                rotation = qmul(
                    qconj(m.bodies[joint.a].q), m.bodies[joint.b].q
                );
                return 0.0;
            }
            const Vec3 axis = normalized(
                rotate(m.bodies[joint.a].q, joint.axis_a)
            );
            const Vec3 point_a = m.bodies[joint.a].r
                + rotate(m.bodies[joint.a].q, joint.pa);
            const Vec3 point_b = m.bodies[joint.b].r
                + rotate(m.bodies[joint.b].q, joint.pb);
            return dot(axis, point_a-point_b);
        };
        coupler.reference_translation_a = coordinate_reference(
            first, coupler.coordinate_a, coupler.reference_rotation_a
        );
        coupler.reference_translation_b = coordinate_reference(
            second, coupler.coordinate_b, coupler.reference_rotation_b
        );
        m.rows += 1;
        coupler.row = m.rows - 1;
        m.coordinate_couplers.push_back(coupler);
    }
    for(std::size_t i=0;i<in.spring_count;++i){
        if (!only_when_no_blocks("spring")) return m;
        Spring s; s.a=in.spring_body_a[i]; s.b=in.spring_body_b[i];
        s.pa={in.spring_point_a[i*3],in.spring_point_a[i*3+1],in.spring_point_a[i*3+2]};
        s.pb={in.spring_point_b[i*3],in.spring_point_b[i*3+1],in.spring_point_b[i*3+2]};
        s.k=in.spring_stiffness[i];
        s.c_compression=in.spring_compression_damping[i];
        s.c_rebound=in.spring_rebound_damping[i];
        s.free_length=in.spring_free_length[i];
        s.minimum_length=in.spring_minimum_length[i];
        s.maximum_length=in.spring_maximum_length[i];
        s.compression_stop_k=in.spring_compression_stop_stiffness[i];
        s.compression_stop_c=in.spring_compression_stop_damping[i];
        s.rebound_stop_k=in.spring_rebound_stop_stiffness[i];
        s.rebound_stop_c=in.spring_rebound_stop_damping[i];
        if (in.spring_damper_curve_count && in.spring_damper_curve_offset &&
            in.spring_damper_curve_velocity && in.spring_damper_curve_force) {
            const int count = in.spring_damper_curve_count[i];
            const int offset = in.spring_damper_curve_offset[i];
            if (count < 0 || offset < 0) {
                error="invalid damper curve range";
                return m;
            }
            if (count == 1) {
                error="a damper curve needs at least two points";
                return m;
            }
            for (int p = 0; p < count; ++p) {
                const double velocity =
                    in.spring_damper_curve_velocity[offset+p];
                const double force = in.spring_damper_curve_force[offset+p];
                if (!std::isfinite(velocity) || !std::isfinite(force)) {
                    error="damper curve must be finite";
                    return m;
                }
                // Strictly increasing in velocity keeps the interpolation
                // single-valued; a non-monotonic force is allowed because real
                // shocks are not monotonic near the blow-off point.
                if (p > 0 && velocity <= s.damper_velocity.back()) {
                    error="damper curve velocity must strictly increase";
                    return m;
                }
                s.damper_velocity.push_back(velocity);
                s.damper_force.push_back(force);
            }
        }
        if(s.a<0||s.b<0||s.a>=static_cast<int>(in.body_count)||s.b>=static_cast<int>(in.body_count)||
           s.k<0||s.c_compression<0||s.c_rebound<0||s.free_length<0||
           s.compression_stop_k<0||s.compression_stop_c<0||s.rebound_stop_k<0||s.rebound_stop_c<0||
           (std::isfinite(s.minimum_length) && s.minimum_length<0)||
           (std::isfinite(s.maximum_length) && s.maximum_length<0)||
           (std::isfinite(s.minimum_length) && std::isfinite(s.maximum_length) &&
            s.minimum_length>=s.maximum_length)){
            error="invalid spring or stop parameters";
            return m;
        }
        m.springs.push_back(s);
    }
    for(std::size_t i=0;i<in.bushing_count;++i){
        if (!only_when_no_blocks("bushing")) return m;
        Bushing b;
        b.a=in.bushing_body_a[i]; b.b=in.bushing_body_b[i];
        b.pa={in.bushing_point_a[i*3],in.bushing_point_a[i*3+1],in.bushing_point_a[i*3+2]};
        b.pb={in.bushing_point_b[i*3],in.bushing_point_b[i*3+1],in.bushing_point_b[i*3+2]};
        const double* qa=&in.bushing_frame_a_quaternion[i*4];
        const double* qb=&in.bushing_frame_b_quaternion[i*4];
        const double* qr=&in.bushing_reference_quaternion[i*4];
        const Quat frame_a{qa[0],qa[1],qa[2],qa[3]};
        const Quat frame_b{qb[0],qb[1],qb[2],qb[3]};
        const Quat reference{qr[0],qr[1],qr[2],qr[3]};
        if (!unit_quaternion(frame_a) || !unit_quaternion(frame_b) ||
            !unit_quaternion(reference)) {
            error="bushing quaternions must be finite and unit length";
            return m;
        }
        b.frame_a=qnormalize(frame_a);
        b.frame_b=qnormalize(frame_b);
        b.reference=qnormalize(reference);
        b.reference_translation={
            in.bushing_reference_translation[i*3],
            in.bushing_reference_translation[i*3+1],
            in.bushing_reference_translation[i*3+2]
        };
        for(int j=0;j<36;++j){
            b.stiffness[static_cast<std::size_t>(j)]=in.bushing_stiffness_6x6[i*36+j];
            b.damping[static_cast<std::size_t>(j)]=in.bushing_damping_6x6[i*36+j];
        }
        for(int j=0;j<6;++j) b.preload[static_cast<std::size_t>(j)]=in.bushing_preload_6[i*6+j];
        if(b.a<0||b.b<0||b.a>=static_cast<int>(in.body_count)||b.b>=static_cast<int>(in.body_count)){
            error="invalid bushing body index"; return m;
        }
        for(int r=0;r<6;++r) for(int c=0;c<6;++c){
            const double ks=b.stiffness[static_cast<std::size_t>(r*6+c)];
            const double cs=b.damping[static_cast<std::size_t>(r*6+c)];
            if(!std::isfinite(ks)||!std::isfinite(cs)){error="bushing matrices must be finite";return m;}
            if(std::abs(ks-b.stiffness[static_cast<std::size_t>(c*6+r)])>1e-10||
               std::abs(cs-b.damping[static_cast<std::size_t>(c*6+r)])>1e-10){
                error="bushing matrices must be symmetric"; return m;
            }
        }
        m.bushings.push_back(b);
    }
    for(std::size_t i=0;i<in.anti_roll_bar_count;++i){
        if (!only_when_no_blocks("anti_roll_bar")) return m;
        AntiRollBar bar;
        bar.a=in.anti_roll_body_a[i]; bar.b=in.anti_roll_body_b[i];
        bar.axis_a={
            in.anti_roll_axis_a[i*3],
            in.anti_roll_axis_a[i*3+1],
            in.anti_roll_axis_a[i*3+2]
        };
        const double* qr=&in.anti_roll_reference_quaternion[i*4];
        const Quat reference{qr[0],qr[1],qr[2],qr[3]};
        if (!unit_quaternion(reference)) {
            error="anti-roll reference quaternion must be finite and unit length";
            return m;
        }
        bar.reference=qnormalize(reference);
        bar.stiffness=in.anti_roll_stiffness[i];
        bar.damping=in.anti_roll_damping[i];
        if(bar.a<0||bar.b<0||bar.a>=static_cast<int>(in.body_count)||bar.b>=static_cast<int>(in.body_count)||
           norm(bar.axis_a)<kEps||bar.stiffness<0||bar.damping<0){
            error="invalid anti-roll bar"; return m;
        }
        bar.axis_a=normalized(bar.axis_a);
        m.anti_roll_bars.push_back(bar);
    }
    for(std::size_t i=0;i<in.tire_count;++i){
        if (!only_when_no_blocks("tire")) return m;
        Tire t;
        t.body=in.tire_body[i];
        t.center={in.tire_center_local[i*3],in.tire_center_local[i*3+1],in.tire_center_local[i*3+2]};
        t.frame_body=t.body;
        t.frame_center=t.center;
        t.spin_axis={
            in.tire_spin_axis_local[i*3],
            in.tire_spin_axis_local[i*3+1],
            in.tire_spin_axis_local[i*3+2]
        };
        t.forward_axis={
            in.tire_forward_axis_local[i*3],
            in.tire_forward_axis_local[i*3+1],
            in.tire_forward_axis_local[i*3+2]
        };
        t.radius=in.tire_radius[i];
        t.maximum_compression=in.tire_maximum_compression[i];
        t.k=in.tire_stiffness[i];
        t.c=in.tire_damping[i];
        t.mu_longitudinal=in.tire_mu_longitudinal[i];
        t.mu_lateral=in.tire_mu_lateral[i];
        t.brush_k_longitudinal=in.tire_brush_stiffness_longitudinal[i];
        t.brush_k_lateral=in.tire_brush_stiffness_lateral[i];
        t.relaxation_length_longitudinal=
            in.tire_relaxation_length_longitudinal[i];
        t.relaxation_length_lateral=
            in.tire_relaxation_length_lateral[i];
        t.detached_relaxation=in.tire_detached_relaxation[i];
        if(t.body<0||t.body>=static_cast<int>(in.body_count)||t.radius<=0||
           t.maximum_compression<=0||t.maximum_compression>=t.radius||
           t.k<0||t.c<0||t.mu_longitudinal<=0||t.mu_lateral<=0||
           norm(t.spin_axis)<kEps||norm(t.forward_axis)<kEps||
           norm(cross(t.spin_axis,t.forward_axis))<kEps||
           t.brush_k_longitudinal<=0||t.brush_k_lateral<=0||
           t.relaxation_length_longitudinal<=0||
           t.relaxation_length_lateral<=0||t.detached_relaxation<=0){
            error="invalid tire parameters";return m;
        }
        t.spin_axis=normalized(t.spin_axis);
        t.forward_axis=normalized(t.forward_axis);
        m.tires.push_back(t);
    }
    if (!audit_constraint_system(m, error)) return m;
    return m;
}


} // namespace axle_kernel
