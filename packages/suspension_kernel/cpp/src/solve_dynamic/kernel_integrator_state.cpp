// The Newton unknown/state mapping, split from the integrator's input
// translation unit at subtask 03 step 6: the linearization cache and the
// pose candidates are integration concerns.

#include "mb_tire_state/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_solve_dynamic/functions.hpp"

namespace axle_kernel {

bool NewtonLinearizationCache::matches(
    const ResidualContext& context, int dimension
) const {
    return valid && dim == dimension &&
        h == context.h && alpha_m == context.alpha_m &&
        alpha_f == context.alpha_f && beta == context.beta &&
        gamma == context.gamma && alpha_m_z == context.alpha_m_z &&
        alpha_f_z == context.alpha_f_z && gamma_z == context.gamma_z;
}


void NewtonLinearizationCache::update_key(
    const ResidualContext& context, int dimension
) {
    dim = dimension;
    h = context.h;
    alpha_m = context.alpha_m;
    alpha_f = context.alpha_f;
    beta = context.beta;
    gamma = context.gamma;
    alpha_m_z = context.alpha_m_z;
    alpha_f_z = context.alpha_f_z;
    gamma_z = context.gamma_z;
    reuse_steps = 0;
    valid = true;
}


void pose_candidate(
    const State& base, const Model& model, const std::vector<double>& dy,
    State& candidate
) {
    candidate = base;
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        candidate.r[bi] += Vec3{dy[6*fi],dy[6*fi+1],dy[6*fi+2]};
        candidate.q[bi] = normalized_continuous(
            qmul(
                qexp({dy[6*fi+3],dy[6*fi+4],dy[6*fi+5]}),
                candidate.q[bi]
            ),
            candidate.q[bi]
        );
    }
}


void pose_candidate_pose_only(
    const State& base, const Model& model, const std::vector<double>& dy,
    State& candidate
) {
    candidate.r = base.r;
    candidate.q = base.q;
    candidate.v = base.v;
    candidate.omega = base.omega;
    // The Maxwell state (A5) is not part of the solver unknown, so it has to be carried
    // over explicitly: without this the workspace copies evaluated the branch force at a
    // permanent z_m = 0 and produced K_dyn*delta*decay on every wheel -- measured as a
    // 1.1 kN ride-mode oscillation in a mode-14 run whose Maxwell-free twin tracks Adams
    // to 0.1 % of Fz.
    candidate.tire_maxwell = base.tire_maxwell;
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        candidate.r[bi] += Vec3{dy[6*fi],dy[6*fi+1],dy[6*fi+2]};
        candidate.q[bi] = normalized_continuous(
            qmul(
                qexp({dy[6*fi+3],dy[6*fi+4],dy[6*fi+5]}),
                candidate.q[bi]
            ),
            candidate.q[bi]
        );
    }
}


State pose_candidate(const State& base, const Model& model, const std::vector<double>& dy) {
    State candidate = base;
    pose_candidate(base, model, dy, candidate);
    return candidate;
}


State state_from_unknown(
    const ResidualContext& ctx, const std::vector<double>& x, std::vector<double>& a_next,
    std::vector<double>& v_next, std::vector<double>& mu,
    std::vector<double>& lambda) {
    const Model& model = *ctx.model;
    const int n = model.ndof;
    const int m = model.rows;
    std::vector<double> dy(n, 0.0);
    std::copy(x.begin(), x.begin()+n, dy.begin());
    State next = pose_candidate(ctx.previous, model, dy);
    v_next.assign(x.begin()+n, x.begin()+2*n);
    a_next.assign(x.begin()+2*n, x.begin()+3*n);
    lambda.assign(x.begin()+3*n, x.begin()+3*n+m);
    mu.assign(x.begin()+3*n+m, x.begin()+3*n+2*m);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        next.v[bi] = {v_next[6*fi],v_next[6*fi+1],v_next[6*fi+2]};
        next.omega[bi] = {v_next[6*fi+3],v_next[6*fi+4],v_next[6*fi+5]};
    }
    resize_tire_states(model, next);
    const int z_offset = 3*n + 2*m;
    const int per_tire = tire_block_width(model);
    for (int i = 0; i < static_cast<int>(model.tires.size()); ++i) {
        read_tire_states(x, z_offset, i, per_tire, next);
    }
    return next;
}


void state_from_unknown(
    const ResidualContext& ctx, const std::vector<double>& x,
    ResidualWorkspace& workspace
) {
    const Model& model = *ctx.model;
    const int n = model.ndof;
    const int m = model.rows;
    std::copy(x.begin(), x.begin()+n, workspace.dy.begin());
    workspace.pose_increment = workspace.dy;
    pose_candidate_pose_only(ctx.previous, model, workspace.dy, workspace.next);
    std::copy(x.begin()+n, x.begin()+2*n, workspace.v_next.begin());
    std::copy(x.begin()+2*n, x.begin()+3*n, workspace.a_next.begin());
    std::copy(x.begin()+3*n, x.begin()+3*n+m, workspace.lambda.begin());
    std::copy(
        x.begin()+3*n+m, x.begin()+3*n+2*m, workspace.mu.begin()
    );
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        workspace.next.v[bi] = {
            workspace.v_next[6*fi], workspace.v_next[6*fi+1],
            workspace.v_next[6*fi+2]
        };
        workspace.next.omega[bi] = {
            workspace.v_next[6*fi+3], workspace.v_next[6*fi+4],
            workspace.v_next[6*fi+5]
        };
    }
    resize_tire_states(model, workspace.next);
    const int z_offset = 3*n + 2*m;
    const int per_tire = tire_block_width(model);
    for (int i = 0; i < static_cast<int>(model.tires.size()); ++i) {
        read_tire_states(x, z_offset, i, per_tire, workspace.next);
    }
}


} // namespace axle_kernel
