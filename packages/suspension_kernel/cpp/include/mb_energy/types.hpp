#pragma once

/// Energy ledger and static-contact control types (MB_ENERGY).
///
/// These are physical intermediates produced while assembling forces and
/// integrating a step: the element and tire layers write them, the solver
/// consumes the override, and the output layer only reports the totals.
/// They deliberately live in their own low level module rather than in
/// `mb_vehicle`: parking them in the vehicle layer made `mb_tire` and
/// `mb_suspension` depend on it, which is what kept the module graph cyclic.

namespace axle_kernel {

struct StaticContactOverride {
    const std::vector<int>* active{nullptr};
    const std::vector<double>* compression{nullptr};
    const std::vector<double>* compression_derivative{nullptr};
    // 静态载荷递增因子。显式外载和保守内部力元共同按同一因子递增。
    double external_load_scale{1.0};
    double internal_force_scale{1.0};
};

struct EnergyRates {
    double external_power{0.0};
    double road_power{0.0};
    double drive_power{0.0};
    double damper_dissipation{0.0};
    double friction_dissipation{0.0};
    double contact_dissipation{0.0};
};

struct EnergyStorage {
    double gravity{0.0};
    double spring{0.0};
    double stop{0.0};
    double bushing{0.0};
    double anti_roll{0.0};
    double tire_normal{0.0};
    double tire_brush{0.0};

    double total() const {
        return gravity+spring+stop+bushing+anti_roll+
            tire_normal+tire_brush;
    }
};

struct EnergyInterval {
    double external_work{0.0};
    double road_work{0.0};
    double drive_work{0.0};
    double damper_dissipation{0.0};
    double friction_dissipation{0.0};
    double contact_dissipation{0.0};

    double total_work() const {
        return external_work+road_work+drive_work;
    }

    double total_physical_dissipation() const {
        return damper_dissipation+friction_dissipation+
            contact_dissipation;
    }
};

} // namespace axle_kernel
