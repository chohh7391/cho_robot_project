#pragma once

#include <cstddef>
#include <deque>

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * Turns a 5 Hz weight stream into the three things a pour actually needs:
 * a mass it can trust, an outflow rate, and an answer to "has it stopped?".
 *
 * No ROS and no clock of its own -- time arrives as a plain double on the
 * host's control clock -- so the whole thing is testable against a synthetic
 * stream without a controller_manager.
 *
 * Measured against the HS-AA on 2026-09-16, which is where the defaults and the
 * warnings below come from:
 *
 *   rate            4.989 Hz, inter-arrival sd 8 ms, max gap 209 ms over 15 min
 *   resolution      0.01 g, and it is really used
 *   static noise    identically zero; no drift over 60 s
 *   during flow     every sample updates; the indicator reports US on 96% of
 *                   them and returns to ST about 1.0 s after flow stops
 *   during drips    the SAME indicator sat still and reported ST for 5.4 s with
 *                   material still arriving
 *
 * That last line is why settled() carries the precondition it does.
 */
class ScaleFilter
{
public:
    struct Sample {
        double grams{0.0};
        //: Measurement time, not arrival time. The indicator's conversion plus
        //: 75 ms of 2400-baud shift register sit between the two.
        double stamp{0.0};
        bool stable{false};
    };

    struct Config {
        //: Largest believable change between two consecutive samples [g]. A
        //: bigger jump is not a pour at any rate -- it is the pan being knocked,
        //: the vessel being lifted, or the front-panel zero being pressed -- and
        //: feeding it to a flow estimate produces a rate no material has.
        double max_step_grams{0.0};
        //: Span the outflow rate is fitted over [s]. One-step differencing
        //: measured sd 1.1 g/s on a 10 g/s pour; a 0.6 s fit roughly halves it.
        double rate_window_sec{0.6};
        //: Samples retained. Only has to cover rate_window_sec.
        std::size_t history{8};
    };

    void configure(const Config & config);
    void reset();

    /**
     * Offer a sample. Returns false if it was rejected as physically
     * impossible, in which case nothing in this filter moved: the previous
     * mass, rate and settle state all still stand.
     *
     * The first sample after a reset is always accepted -- there is nothing to
     * compare it against -- which is deliberate: it is the baseline reading, and
     * a pour that cannot establish one has to fail loudly rather than start
     * from a number the filter invented.
     */
    bool push(const Sample & sample);

    [[nodiscard]] bool has_sample() const { return !history_.empty(); }
    [[nodiscard]] double grams() const;
    [[nodiscard]] double stamp() const;
    [[nodiscard]] bool stable() const;

    //: Least-squares outflow over rate_window_sec [g/s]. Zero with fewer than
    //: two samples in the window, and never negative-clamped: material leaving
    //: the pan is a real event and hiding it would hide a vessel being lifted.
    [[nodiscard]] double flow_rate() const;

    //: Seconds since the newest accepted sample was MEASURED.
    [[nodiscard]] double age(double now) const;

    /**
     * True when the indicator reports stable AND the value has not moved for
     * hold_sec.
     *
     * PRECONDITION: the caller must already know the flow is stopped, by having
     * put the vessel back to an attitude that cannot pour. This is a statement
     * about the indicator's motion detector, not about the pour. Asked while the
     * vessel is still tilted it answers confidently and wrongly: at drip rates
     * the HS-AA held one value, flagged stable, for 5.4 s with material still
     * arriving, and no choice of hold_sec fixes that -- the gap between
     * avalanches has no upper bound.
     */
    [[nodiscard]] bool settled(double now, double hold_sec) const;

    //: Rejected samples since the last accepted one.
    //:
    //: A rejected sample never re-seeds the filter, so a jump that STAYS -- the
    //: level really changed -- is rejected sample after sample until the newest
    //: accepted one goes stale and the pour stops. That is deliberate. Adopting
    //: the new level would be right for a clump that landed at once, and exactly
    //: wrong for a receiving vessel lifted off the pan: the pour would read a
    //: large negative amount poured and keep pouring onto the bare pan. The pour
    //: cannot tell those apart, so it stops, and this count is what lets it say
    //: why instead of reporting a silent scale.
    [[nodiscard]] int consecutive_rejects() const { return consecutive_rejects_; }
    //: The jump the latest rejected sample would have made [g], 0 when the
    //: latest sample was accepted.
    [[nodiscard]] double last_rejected_step() const { return last_rejected_step_; }

private:
    Config config_;
    std::deque<Sample> history_;
    //: Measurement time of the newest sample whose value differed from the one
    //: before it. This, not the sample count, is what hold_sec is measured from.
    double last_change_stamp_{0.0};
    int consecutive_rejects_{0};
    double last_rejected_step_{0.0};
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
