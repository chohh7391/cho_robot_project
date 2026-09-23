#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "cho_controller_fr5/pour/pour_types.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * Configuration for the shaping law. The gains are the ones the FR5 bringup
 * shipped with before the phase machine existed, which came from the same
 * source as the law.
 */
struct ShapingConfig {
    //: The law's OWN sample period, not the controller's. The reference
    //: implementation runs at 25 Hz, and a law sampled at a different rate is a
    //: different law: the kernel's taps are spaced by exactly this.
    double control_period{0.04};
    //: PD on the weight error, in wrist angular velocity per gram.
    double kp{0.008};
    double kd{0.002};
    //: The shaping function s(t) = exp(-decay*t) * sin(2*pi*freq*t), sampled at
    //: control_period and truncated at kernel_horizon.
    double kernel_horizon{2.5};
    double shaping_freq{0.8};
    double shaping_decay{1.2};
    //: Blend between two normalisations of the kernel. The absolute-sum term
    //: fixes the total response to a step of error; the peak term keeps a single
    //: large sample from being smoothed into nothing.
    double kernel_alpha{0.15};
    //: Wrist angular velocity ceiling [rad/s], and the rate the vessel is
    //: returned at.
    double max_tilt_rate{0.5};
    double container_tolerance{0.5};
    double verify_timeout{8.0};
    //: How long the reading must hold still before the delivered amount is
    //: believed, and how long to wait for that.
    double settle_hold_sec{1.2};
    double settle_timeout{10.0};
    double stall_timeout{3.0};
    double no_flow_epsilon{0.15};
    double tilt_epsilon{0.002};

    [[nodiscard]] bool validate(std::string & why) const;
};

/**
 * Yoshikawa et al.'s chemistry-lab pouring law, as a PourLaw.
 *
 * "we use a shaping function s(t) to guide the direction and frequency of this
 * oscillatory pouring behavior, while a PD controller lowers the pouring error.
 * The end-effector velocity vector is computed by convolving the shaping
 * function s(t) over the PD control signal, v_PD(t) = kp*e + kd*e_dot, where
 * e(t) = x_ref - x_fb."  -- Chemistry Lab Automation via Constrained Task and
 * Motion Planning, arXiv:2212.09672
 *
 * WHY THE OSCILLATION. The paper's argument is not that oscillating is elegant;
 * it is that their scale is 3 s behind. A chemist facing that delay pours a
 * little, waits to see what happened, and pours again, and the shaping kernel
 * makes a continuous law do the same thing without any phase machine. They
 * measured it: 8.1% relative error on water against 81.4% for the same PD with
 * the kernel removed. The kernel is doing almost all of the work.
 *
 * WHY IT IS HERE. This cell's scale is not that scale -- it settles in 1.0 s and
 * streams at 5 Hz with 8 ms of jitter -- so the premise the kernel was built for
 * is weaker here, and whether it still wins is an empirical question nobody has
 * answered on this rig. Both laws are therefore selectable, share the container
 * gate, and run through the same harness.
 *
 * WHAT IT DOES NOT DO, deliberately, because the paper's law does not:
 *   - it never parks below the onset to take a reading, so every measurement it
 *     acts on is taken while the vessel is still pouring;
 *   - it knows nothing about the material beyond its gains;
 *   - it has no notion of a dose quantum, so a tolerance finer than the material
 *     can be metered in is chased rather than floored.
 * Those are the differences under test, not defects to be patched out. Patch
 * them and there is nothing left to compare.
 */
class ShapingPourLaw : public PourLaw
{
public:
    bool configure(const ShapingConfig & config, std::string & why);

    void begin(const PourRequest & request, double now) override;
    void cancel() override;
    PourCommand update(const PourObservation & observation) override;

    [[nodiscard]] const PourReport & report() const override { return report_; }
    [[nodiscard]] double settle_hold() const override { return config_.settle_hold_sec; }
    [[nodiscard]] double baseline_grams() const override { return baseline_; }
    [[nodiscard]] PourPhase phase() const override { return phase_; }
    [[nodiscard]] const char * name() const override { return "shaping"; }
    [[nodiscard]] double return_tilt_rate() const override { return rate_limit(); }

    //: The sampled shaping function, for a test that wants to look at it.
    [[nodiscard]] const std::vector<double> & kernel() const { return kernel_; }

private:
    void build_kernel();
    //: One law sample: PD on the weight error, convolved with the kernel, rate
    //: limited.
    double shaped_velocity(double error);
    PourCommand emit(double tilt_rate) const;
    PourCommand finish(bool success, const std::string & message);
    PourCommand stop_and_return(const std::string & failure, const PourObservation & obs);
    [[nodiscard]] double poured(const PourObservation & obs) const;
    [[nodiscard]] double rate_limit() const;

    ShapingConfig config_;
    bool configured_{false};

    PourRequest request_;
    PourReport report_;

    std::vector<double> kernel_;
    std::vector<double> pd_history_;   // circular, oldest overwritten
    std::size_t pd_head_{0};
    std::size_t pd_count_{0};
    double prev_error_{0.0};
    bool pd_primed_{false};
    double law_accumulator_{0.0};
    double commanded_rate_{0.0};

    PourPhase phase_{PourPhase::Done};
    double start_time_{0.0};
    double last_now_{0.0};
    double baseline_{0.0};
    double grams_at_stop_{0.0};
    double settle_deadline_{0.0};
    double stall_since_{0.0};
    bool stalled_{false};
    bool cancel_requested_{false};
    std::string pending_failure_;
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
