#pragma once

/// The PAC2002 turn-slip state (MB_TIRE_PAC2002).
///
/// It carries the turn-slip force and moment and the travel sign, in both the
/// scalar and the derivative-carrying form.

#include "mb_base/dual.hpp"

namespace axle_kernel {

struct Pac2002TurnSlip {
    double force{0.0};
    double moment{0.0};
    double travel_sign{1.0};
};

struct Pac2002TurnSlipDirectional {
    DirectionalScalar force{};
    DirectionalScalar moment{};
    double travel_sign{1.0};
};

/// Everything the PAC2002 turn-slip channel of the tire loop measures in one
/// pass: the state itself plus the four scalars the observer output reports
/// beside it.  It is returned by value from `pac2002_turn_slip_diagnostics`,
/// which the loop that fills the force state calls from another translation
/// unit, so the type has to be complete here.
struct Pac2002TurnSlipDiagnostics {
    Pac2002TurnSlip turn_slip{};
    double total_spin_rate{0.0};
    double drive{0.0};
    double camber_term{0.0};
    double yaw_rate{0.0};
};

} // namespace axle_kernel
