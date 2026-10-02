#pragma once

/// The optional closed-loop controller ledger (p5-04): one row per sample
/// carrying the slip the ABS law measured, the slip it targeted, the
/// normalized demand it derived, and the driver signal the sample carried.
///
/// The channel is *off* unless `SUSPENSION_KERNEL_CONTROLLER_OUTPUT` asks for
/// it.  While it is off nothing is allocated, described or written, so the
/// default path's result document and blob stay the ones they always were; the
/// switch only decides whether the observer records what it already computes.
///
/// The recorder lives in `mb_config` because every layer that has to reach it
/// (element, output, abi) already includes that module, so the channel adds no
/// edge to the module graph and needs no kernel type of its own: the row holds
/// four plain doubles the element law has already computed.
///
/// A record's `kControllerOutputWidth` columns are: [0] the measured slip, [1]
/// the target slip (-1 when no law ran), [2] the normalized demand the law
/// derived, [3] the normalized driver demand the sample carried before the law
/// replaced it.  A sample no law ran in leaves its row NaN, which is what "no
/// record" means.
///
/// The `measured_slip` column is the tire's *longitudinal slip* as the solver's
/// `State::tire_sx` carries it -- a dimensionless slip ratio, not a speed.  It
/// is the same quantity `kernel_integrator.cpp` forms and the same one the
/// element law's sign branch reads, so the control half and the state half of
/// one run are the same measurement rather than two derivations of it.
///
/// The driver demand of the open row lives on the sink rather than in the
/// element law because the law has no per-sample state of its own: the
/// `rotational_torque` family is evaluated inside the solver's residual, and a
/// residual evaluation is a pure function of the state it is handed.  The law
/// therefore states the driver demand it read, then records the row; a sample
/// nothing stated it for records no driver column.

#include "mb_config/constants.hpp"

#include <cstddef>
#include <limits>

namespace axle_kernel {

/// The value the driver-demand column takes when no element law stated one.
/// It is the same "no record" the untouched block columns carry.
inline constexpr double kNoDriverDemand =
    std::numeric_limits<double>::quiet_NaN();

/// The process-wide recorder.  It is a single object because the target block
/// is process state: the model, the case and the output buffers are not visible
/// to the element laws, and the channel must not change any of their
/// signatures.
class ControllerSink {
  public:
    /// Bind the sink to one case's block slice.  A null block and a sample
    /// count of zero both leave the sink inactive.
    void configure(double* block, std::size_t samples) {
      block_ = block;
      samples_ = samples;
      row_ = nullptr;
      driver_demand_ = kNoDriverDemand;
    }

    /// Forget the target without touching the block itself.
    void clear() { configure(nullptr, 0); }

    bool configured() const { return block_ != nullptr && samples_ != 0; }

    /// Whether a sample's recording window is open.  The element law asks this,
    /// so a pass that is not recording costs one pointer test.
    bool recording() const { return row_ != nullptr; }

    /// Open the recording window for one sample.  A sample outside the block
    /// leaves the window shut rather than addressing a row that is not there.
    void begin_sample(std::size_t sample) {
      row_ = configured() && sample < samples_
          ? block_ + sample * kControllerOutputWidth
          : nullptr;
      driver_demand_ = kNoDriverDemand;
    }

    void end_sample() {
      row_ = nullptr;
      driver_demand_ = kNoDriverDemand;
    }

    /// State the driver demand of the open sample, before a law replaced it.
    void set_driver_demand(double demand) { driver_demand_ = demand; }

    /// The driver demand this sample carried, or NaN when nothing said.
    double driver_demand() const { return driver_demand_; }

    /// Write one row.  `target` is -1 when no law ran, which is the value the
    /// frozen column order documents for "no law"; the driver column is the
    /// demand the sample carried before the law replaced it.
    void record(double measured, double target, double demand) {
      if (row_ == nullptr) return;
      row_[0] = measured;
      row_[1] = target;
      row_[2] = demand;
      row_[3] = driver_demand_;
    }

  private:
    double* block_ = nullptr;
    std::size_t samples_ = 0;
    /// This sample's row.  Null when no window is open, which is what makes a
    /// physics pass silent.
    double* row_ = nullptr;
    /// The open sample's driver demand; NaN until an element law states one.
    double driver_demand_ = kNoDriverDemand;
};

/// The process sink when the switch is on, and null when it is off.
ControllerSink* controller_sink();

/// Whether the process sink is recording a sample.  This is what the element
/// law calls, so a pass that is not recording pays one boolean test.
bool controller_sink_active();

/// Record one sample of the closed loop.  A no-op unless the sink is on and a
/// window is open, so the element law can call it unconditionally.
void record_controller_sample(double measured, double target, double demand);

/// State the driver demand of the sample currently being recorded.  A no-op
/// when nothing is recording.
void set_controller_driver_demand(double demand);

} // namespace axle_kernel
