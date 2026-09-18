/// The PAC2002 capability declaration: what this kernel computes exactly and,
/// just as important, which coefficient families it refuses.
///
/// The authoring layer has to reject a tire that requests a feature the kernel
/// does not implement, because the alternative is a result that silently omits
/// it.  That decision needs the kernel's scope, and the scope used to be copied
/// into the authoring layer by hand.  A copy drifts, and the drift is silent in
/// the direction that matters: a tire the kernel *could* run gets rejected, or
/// worse, one it cannot run gets through.  The declaration lives here, beside
/// the mode tables it describes, and reaches the authoring layer through
/// `suspension_kernel_capabilities`.
///
/// The kernel is the only thing that can know this.  A coefficient the ABI
/// payload has no slot for is refused by construction, but several of the names
/// below *do* have slots -- QBRP2/QDTP2 are carried and no documented factor
/// reads them, DYNAMIC_STIFFNESS/DYNAMIC_DAMPING feed the Maxwell element and
/// BP3/BP4 are the third and fourth moment relaxation factors -- so "is it in
/// the payload?" is not the same question as "does the kernel evaluate it?".

#include "mb_tire/pac2002/functions.hpp"

#include <algorithm>

namespace axle_kernel {

const std::vector<int>& pac2002_supported_use_modes() {
    // Mode 25 carries the turn-slip relaxation set of Eq3963-Eq3966, the
    // composite channels of Eq3969-Eq3970, the steady-state spin/parking
    // factors of Eq3329-Eq3356 and the yaw coupling of Eq3961/Eq3967.
    static const std::vector<int> kModes{
        0, 1, 2, 3, 4, 11, 12, 13, 14, 23, 24, 25,
    };
    return kModes;
}

const std::vector<const char*>& pac2002_refused_parameters() {
    // The Maxwell non-rolling vertical element.  `USE_DYNAMIC_STIFFNESS` has its
    // own feature flag below; these two are unquoted numbers, so the importer's
    // generic extractor does copy them and they are checked here as well.
    static const std::vector<const char*> kNames{
        "DYNAMIC_STIFFNESS",
        "DYNAMIC_DAMPING",
    };
    return kNames;
}

const std::vector<const char*>& pac2002_refused_feature_flags() {
    static const std::vector<const char*> kNames{
        "PAC2002_UNSUPPORTED_BELT_DYNAMICS",
        "PAC2002_UNSUPPORTED_CONTACT_MODEL",
        "PAC2002_UNSUPPORTED_DYNAMIC_STIFFNESS",
        "PAC2002_UNSUPPORTED_FE_METHOD",
        "PAC2002_UNSUPPORTED_FITTYP",
        "PAC2002_UNSUPPORTED_LOCAL_SOLVER",
        "PAC2002_UNSUPPORTED_PAC_MC",
    };
    return kNames;
}

const std::vector<Pac2002RefusedFamily>& pac2002_refused_families() {
    // QBRP2 and QDTP2 appear in the ABI enum but no documented factor reads
    // them (Eq3347 gives DDrgamma from QDTP1 alone), so a tire that sets either
    // fails closed rather than being solved with a term dropped.
    static const Pac2002RefusedFamily kTurnSlipSecondOrder{
        "turn_slip_second_order_coefficients",
        "unsupported second-order turn-slip coefficients",
        {"QBRP2", "QDTP2"},
    };
    // [DYNAMIC_COEFFICIENTS] and [CONTACT_COEFFICIENTS] carry the non-linear
    // (advanced) transient contact-mass model.  MC/KX/KY/CX/CY drive the contact
    // patch's force balance, CXZ*/CXX*/CYZ*/CYY* scale its carcass stiffness,
    // PA1/PA2 give the half contact length, IC/KP/CP are the yaw degree of
    // freedom and EP/EP12/BF2/BP1/BP2 are the turn-slip relaxation set the
    // USE_MODE 25 kernel evaluates.  What is left is BP3/BP4 -- relaxation
    // factors no documented filter uses -- and the enveloping helpers (N_*/PAE/
    // PB*/PBE/PCE/PLS) of the non-point contact models.
    static const Pac2002RefusedFamily kContactMass{
        "contact_mass_advanced_transient",
        "unsupported non-linear transient contact-mass coefficients",
        {
            "BP3", "BP4", "N_LENGTH", "N_WIDTH", "PAE", "PB1", "PB2", "PB3",
            "PBE", "PCE", "PLS",
        },
    };
    // [BELT_PARAMETERS] carries the rigid-ring belt model (rim-to-belt six
    // degree of freedom bushing) of "PAC2002 with Belt Dynamics".
    static const Pac2002RefusedFamily kBeltDynamics{
        "belt_dynamics",
        "unsupported belt dynamics coefficients",
        {
            "QBVTH", "QBVXZ", "QCBGM", "QCBTH", "QCBXZ", "QCBY", "QCCFI",
            "QCCX", "QCCY", "QIBXZ", "QIBY", "QIC", "QKBGM", "QKBTH", "QKBXZ",
            "QKBY", "QKCFI", "QKCX", "QKCY", "QMB", "QMC", "TYRE_MASS",
        },
    };
    static const std::vector<Pac2002RefusedFamily> kFamilies{
        kTurnSlipSecondOrder,
        kContactMass,
        kBeltDynamics,
    };
    return kFamilies;
}

} // namespace axle_kernel
