#pragma once

/// What the tire assembler decided when the normal force was not positive.
///
/// The loop that carries the tire force state used to hold this decision inline;
/// K4 split the block into `handle_non_positive_normal_force`, and the two
/// outcomes -- skip the rest of this tire, and leave the loop entirely in
/// brush-only mode -- became a returned value rather than two `continue`s.  The
/// type lives beside the assembler that returns it.

namespace axle_kernel {

struct NonPositiveNormalForceResult {
    bool skip_rest{false};
    bool brush_only_exit{false};
};

} // namespace axle_kernel
