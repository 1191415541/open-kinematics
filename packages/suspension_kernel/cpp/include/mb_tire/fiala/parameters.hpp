#pragma once

/// The Fiala parameter table's slot indices (MB_TIRE_FIALA).

#include <cstddef>

namespace axle_kernel {

enum FialaParameterIndex {
    FIALA_CSLIP = 0,
    FIALA_CALPHA = 1,
    // Adams' Fiala property format defines CGAMMA but its handling force model
    // ignores it ("Camber angle has no effect on tire forces"), so the slot is
    // carried through for fidelity and never read by the force law.
    FIALA_CGAMMA = 2,
    // Reserved.  MGAMMA and the two damping slots are not keywords of the Adams
    // Fiala format at all; they exist only to keep the 14-slot layout stable.
    FIALA_RESERVED_3 = 3,
    // The tire's [MODEL] USE_MODE.  This slot used to hold CSPIN, which the Adams
    // Fiala format does not define, so nothing ever read it -- which is why it
    // could be repurposed without an ABI change.
    FIALA_USE_MODE = 4,
    FIALA_UMIN = 5,
    FIALA_UMAX = 6,
    FIALA_RELAX_LENGTH_X = 7,
    FIALA_RELAX_LENGTH_Y = 8,
    FIALA_WIDTH = 9,
    FIALA_ROLLING_RESISTANCE = 10,
    FIALA_LOW_SPEED_THRESHOLD = 11,
    FIALA_RESERVED_12 = 12,
    FIALA_RESERVED_13 = 13
};

} // namespace axle_kernel
