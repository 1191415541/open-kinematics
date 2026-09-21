#pragma once

/// The optional element-wrench channel (05 step 3): one record per (element,
/// end) and sample, carrying the wrench the element actually applied to one
/// body.
///
/// The channel is *off* unless `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT` asks
/// for it.  While it is off nothing is allocated, described or written, so the
/// default path's result document and blob stay the ones they always were; the
/// switch only decides whether the observer records what it already computes.
///
/// The recorder lives in `mb_config` because every layer that has to reach it
/// (element, force, tire, output, abi) already includes this module, so the
/// channel adds no edge to the module graph and needs no kernel type of its
/// own: the row arithmetic runs on the plain doubles `add_force_on_body` has
/// already computed.
///
/// A record's `kElementWrenchOutputWidth` columns are: [0..2] world force (N),
/// [3..5] world moment about the receiving body's origin (N*m), [6] element
/// type code, [7..9] world coordinates of the point the force acts at, [10]
/// body a, [11] body b, [12] the body the record belongs to.
///
/// Rows are reserved by index rather than filled in call order, so an element
/// that applies nothing in a sample leaves its own rows unset (NaN) instead of
/// shifting every later element's row.

#include "mb_config/constants.hpp"

#include <cmath>
#include <cstddef>

namespace axle_kernel {

/// The `element` column: which law applied the wrench.  These are the channel's
/// frozen codes, not a model-layer enum.
enum ElementWrenchType {
  kElementWrenchSpring = 1,
  kElementWrenchBushing = 2,
  kElementWrenchAntiRoll = 3,
  kElementWrenchSteering = 4,
  kElementWrenchDriveBrake = 5,
  kElementWrenchTire = 6,
  kElementWrenchExternal = 7,
};

/// The element counts a sample's rows are laid out from.  They are the counts
/// of the model the solver runs, so the ABI (which sizes the block) and the
/// observer (which addresses a row) derive the same shape from the same
/// numbers.
struct ElementWrenchCounts {
  std::size_t springs = 0;
  std::size_t bushings = 0;
  std::size_t anti_rolls = 0;
  std::size_t steering = 0;
  std::size_t tires = 0;
  std::size_t bodies = 0;
  std::size_t drags = 0;
};

/// Records per sample: two per spring, bushing, anti-roll bar and steering
/// actuator (one per end), one per tire for its contact wrench, four per tire
/// for its drive/brake torques (drive, its reaction, brake, its reaction), and
/// one per body plus one per aerodynamic drag for the external sources.  A
/// body's row carries its declared wrench and its gravity; a drag's row carries
/// the drag force at its application point.
std::size_t element_wrench_record_count(const ElementWrenchCounts& counts);

/// The process-wide recorder.  It is a single object because the row layout and
/// the target block are process state: the model, the case and the output
/// buffers are not visible to the element laws, and the channel must not change
/// any of their signatures.
class ElementWrenchSink {
  public:
    /// Bind the sink to one case's block slice.  A null block and a record_count
    /// of zero both leave the sink inactive.
    void configure(double* block, std::size_t record_count) {
      block_ = block;
      record_count_ = record_count;
      rows_ = nullptr;
      row_ = nullptr;
    }

    /// Forget the target without touching the block itself.
    void clear() { configure(nullptr, 0); }

    bool configured() const { return block_ != nullptr && record_count_ != 0; }

    /// Whether a sample's recording window is open.  Every law that applies a
    /// wrench asks this, so a pass that is not recording costs one test.
    bool recording() const { return rows_ != nullptr; }

    /// Open the recording window for one sample.  The counts have to describe
    /// the same shape the block was sized for; a mismatch leaves the window
    /// shut rather than addressing rows that are not there.
    void begin_sample(std::size_t sample, const ElementWrenchCounts& counts);

    void end_sample() { rows_ = nullptr; row_ = nullptr; }

    /// Select the row reserved for (element index, end) of `type`.  The
    /// identity columns are written here, the action point is offered as the
    /// row's point until an applied force names its own, and the force and
    /// moment columns are left for `add_*`: a row nothing is added to stays
    /// NaN, which is what "this element applied nothing to this body" means.
    /// A negative body is not a body, and selects no row.
    void open(ElementWrenchType type, std::size_t index, std::size_t end,
              int body_a, int body_b, int body,
              double point_x, double point_y, double point_z);

    /// Add the wrench of a force applied at `origin + arm`: the force itself,
    /// its moment about the body origin, and its action point.
    template <class V>
    void add_force(const V& force, const V& arm, const V& origin) {
      if (row_ == nullptr) return;
      add_slot(row_ + 0, force.x);
      add_slot(row_ + 1, force.y);
      add_slot(row_ + 2, force.z);
      add_slot(row_ + 3, arm.y*force.z - arm.z*force.y);
      add_slot(row_ + 4, arm.z*force.x - arm.x*force.z);
      add_slot(row_ + 5, arm.x*force.y - arm.y*force.x);
      row_[7] = origin.x + arm.x;
      row_[8] = origin.y + arm.y;
      row_[9] = origin.z + arm.z;
    }

    /// Add a pure moment about the body origin.  It has no application point,
    /// so the row keeps the point it was opened with.
    template <class V>
    void add_torque(const V& torque) {
      if (row_ == nullptr) return;
      add_slot(row_ + 3, torque.x);
      add_slot(row_ + 4, torque.y);
      add_slot(row_ + 5, torque.z);
    }

    /// Add a wrench the caller assigned straight into the force and torque
    /// buffers (an external body wrench, gravity).  There is no lever to
    /// derive a point from -- the row is opened with the point it acts at.
    template <class V>
    void add_wrench(const V& force, const V& torque) {
      if (row_ == nullptr) return;
      add_slot(row_ + 0, force.x);
      add_slot(row_ + 1, force.y);
      add_slot(row_ + 2, force.z);
      add_slot(row_ + 3, torque.x);
      add_slot(row_ + 4, torque.y);
      add_slot(row_ + 5, torque.z);
    }

  private:
    /// The row reserved for (type, element index, end), or null when the sink
    /// is not recording, the type is unknown or the row is outside the sample.
    double* row_at(ElementWrenchType type, std::size_t index, std::size_t end) const;

    /// Accumulate into a column: an untouched column is NaN, and the first
    /// contribution replaces it so a genuinely applied zero stays a zero.
    static void add_slot(double* slot, double value) {
      *slot = std::isnan(*slot) ? value : *slot + value;
    }

    double* block_ = nullptr;
    std::size_t record_count_ = 0;
    /// First row of each type's group within a sample, indexed by type code.
    std::size_t group_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    /// This sample's first row, and the row currently selected.  Both null when
    /// no window is open, which is what makes a physics pass silent.
    double* rows_ = nullptr;
    double* row_ = nullptr;
};

/// The process sink when the switch is on, and null when it is off.
ElementWrenchSink* element_wrench_sink();

/// The process sink while it is recording a sample, and null otherwise.  This
/// is what the element laws and the ABI call, so a pass that is not recording
/// pays one pointer test.
ElementWrenchSink* active_element_wrench_sink();

} // namespace axle_kernel
