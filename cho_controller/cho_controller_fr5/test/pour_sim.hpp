#pragma once
// The synthetic vessel every pour law is tested and compared against.
//
// Shared by test_pour.cpp and pour_ab.cpp so that the numbers quoted in
// controllers.yaml for the two laws come from the same vessel the tests pin.
//
// It is NOT a physical model, and it is not neutral: the phase machine was
// developed against it, while the shaping law only had its two gains swept
// here. Read any comparison it produces with that bias in mind, and settle the
// question on the rig.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "cho_controller_fr5/pour/pour_types.hpp"
#include "cho_controller_fr5/pour/scale_filter.hpp"

namespace pour_sim {

/**
 * The tilt at which a straight-walled cylinder of water just reaches its rim,
 * for a given amount left in it: the physics a real beaker adds that a fixed
 * onset leaves out. As the vessel empties the onset RISES -- for a 100 mL
 * beaker (50 mm bore, 70 mm to the rim) from 0.65 rad at the 100 mL mark to
 * 0.94 rad after 30 g has gone -- so a law that finds the onset once and keeps
 * it is aiming at an angle that has moved.
 *
 * Tilting toward the lip, the free surface through the lip is
 * z = H - (R - x) tan(theta) in the vessel frame. Until it reaches the bottom
 * the water is a slanted-topped cylinder of volume A (H - R tan theta); after,
 * it is the wedge under that plane, integrated numerically. Spout ignored.
 */
class CylinderOnset
{
public:
    CylinderOnset() = default;
    CylinderOnset(double radius_mm, double height_mm) : r_(radius_mm), h_(height_mm)
    {
        // Onset for every 0.5 g from empty to brim, by bisection, once.
        const double brim = kPi * r_ * r_ * h_ / 1000.0;
        for (double g = 0.0; g <= brim + 0.5; g += 0.5) {
            grams_.push_back(g);
            onset_.push_back(invert(g * 1000.0));
        }
    }

    [[nodiscard]] bool enabled() const { return !grams_.empty(); }

    //: Onset [rad] with `grams` of water left in the vessel.
    [[nodiscard]] double onset(double grams) const
    {
        if (grams <= grams_.front()) {
            return onset_.front();
        }
        if (grams >= grams_.back()) {
            return onset_.back();
        }
        const std::size_t i = static_cast<std::size_t>(grams / 0.5);
        const double f = (grams - grams_[i]) / 0.5;
        return onset_[i] + f * (onset_[i + 1] - onset_[i]);
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    [[nodiscard]] double volume_at_spill(double theta) const
    {
        const double t = std::tan(theta);
        if (h_ - 2.0 * r_ * t >= 0.0) {
            return kPi * r_ * r_ * (h_ - r_ * t);
        }
        const double x0 = r_ - h_ / t;
        constexpr int n = 400;
        double v = 0.0;
        for (int i = 0; i < n; ++i) {
            const double x = x0 + (r_ - x0) * (i + 0.5) / n;
            v += 2.0 * std::sqrt(std::max(r_ * r_ - x * x, 0.0)) * t * (x - x0);
        }
        return v * (r_ - x0) / n;
    }

    [[nodiscard]] double invert(double volume_mm3) const
    {
        double lo = 1e-6, hi = kPi / 2.0 - 1e-6;
        for (int k = 0; k < 60; ++k) {
            const double mid = 0.5 * (lo + hi);
            if (volume_at_spill(mid) > volume_mm3) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return 0.5 * (lo + hi);
    }

    double r_{0.0};
    double h_{0.0};
    std::vector<double> grams_;
    std::vector<double> onset_;
};

using cho_controller::fr5::pour::PourCommand;
using cho_controller::fr5::pour::PourLaw;
using cho_controller::fr5::pour::PourObservation;
using cho_controller::fr5::pour::PourPhase;
using cho_controller::fr5::pour::PourRequest;
using cho_controller::fr5::pour::ScaleFilter;

/**
 * A vessel that pours. Deliberately NOT the model the planner uses: the planner
 * knows a transport delay and a tail, this knows an onset angle, a flow that
 * grows with tilt, a pipe with a delay in it, and a tail that drains on its own
 * schedule. If the two agreed the tests would only be checking arithmetic.
 */
struct VesselSim {
    double onset_tilt{0.15};
    double gain{40.0};          // g/s per rad above onset
    double transport_delay{0.4};
    double tail_grams{0.25};    // released after the flow stops
    double tail_tau{0.3};

    //: Mass arriving on the pan from somewhere other than the lip [g/s] --
    //: a drip off the gripper, a leak -- from leak_after on. It is what makes a
    //: park refuse to settle however low it goes.
    double leak_gps{0.0};
    double leak_after{1e9};

    //: When enabled, the onset follows what is left in the vessel instead of
    //: staying at onset_tilt, and the vessel can run dry.
    CylinderOnset cylinder;
    double contents_grams{0.0};

    double landed{0.0};
    //: Everything that has left the lip, landed or not.
    double poured_out{0.0};
    double tail_pending{0.0};
    bool was_flowing{false};
    std::deque<std::pair<double, double>> pipe;  // (arrival time, grams)

    [[nodiscard]] double current_onset() const
    {
        return cylinder.enabled() ? cylinder.onset(contents_grams - poured_out) : onset_tilt;
    }

    double flow_at(double tilt) const
    {
        return std::max(0.0, (tilt - current_onset())) * gain;
    }

    void step(double now, double dt, double tilt)
    {
        if (now >= leak_after) {
            landed += leak_gps * dt;
        }
        double flow = flow_at(tilt);
        if (cylinder.enabled()) {
            flow = std::min(flow, std::max(0.0, contents_grams - poured_out) / dt);
        }
        if (flow > 0.0) {
            pipe.emplace_back(now + transport_delay, flow * dt);
            poured_out += flow * dt;
            was_flowing = true;
        } else if (was_flowing) {
            // The stream broke: whatever was clinging to the lip now drains.
            tail_pending += tail_grams;
            was_flowing = false;
        }
        if (tail_pending > 0.0) {
            const double released = std::min(tail_pending, tail_pending * dt / tail_tau + 1e-9);
            tail_pending -= released;
            pipe.emplace_back(now + transport_delay, released);
        }
        while (!pipe.empty() && pipe.front().first <= now) {
            landed += pipe.front().second;
            pipe.pop_front();
        }
    }
};

struct RunResult {
    bool finished{false};
    bool success{false};
    std::string message;
    double delivered{0.0};
    double peak_tilt{0.0};
    int trim_pulses{0};
    double seconds{0.0};
    PourPhase last_phase{PourPhase::Verify};
    std::vector<PourPhase> phases_seen;
    //: Most negative tilt reached: how far behind the carried attitude.
    double min_tilt{0.0};
    //: Tilt at every entry into Settle, i.e. every park actually reached.
    std::vector<double> park_angles;
};

struct RunOptions {
    //: Tilt the vessel is handed over at, in pour-positive coordinates. Nonzero
    //: models a beaker gripped off-square.
    double grasp_tilt{0.0};
    double sim_transport_delay{0.4};
    double sim_tail{0.25};
    double sim_gain{40.0};
    double sim_onset{0.15};
    //: Stop publishing scale samples after this many seconds.
    double scale_dies_at{1e9};
    double horizon{240.0};
    double leak_gps{0.0};
    double leak_after{1e9};
    //: A joint limit, in the planner's tilt coordinates: the arm cannot be
    //: commanded below this, whatever the planner asks for.
    double tilt_floor{-1e9};
    double max_back_tilt{0.3};
    //: What the vessel on the pan actually weighs. NaN means the one the goal
    //: describes; anything else is a goal describing the wrong vessel.
    double vessel_grams{std::nan("")};
    //: A real cylinder instead of a fixed onset: bore radius and rim height
    //: [mm], and how much water it starts with [g]. Radius 0 keeps sim_onset.
    double cylinder_radius_mm{0.0};
    double cylinder_height_mm{0.0};
    double contents_grams{0.0};
    //: Print the run every this many seconds; 0 is silent.
    double trace_every{0.0};
};

/**
 * Drive the planner the way PouringController does: integrate its tilt rate at
 * the control rate, quantise and sample the pan at 5 Hz, hand back mass, rate
 * and settle state.
 */
inline RunResult run_pour(PourLaw & planner, const PourRequest & goal, const RunOptions & opt)
{
    // The controller fills max_back_tilt from its own parameter; so does this.
    PourRequest request = goal;
    request.max_back_tilt = opt.max_back_tilt;

    constexpr double kControlDt = 1.0 / 125.0;
    constexpr double kScaleDt = 0.2;
    constexpr double kResolution = 0.01;

    VesselSim vessel;
    vessel.onset_tilt = opt.sim_onset;
    vessel.gain = opt.sim_gain;
    vessel.transport_delay = opt.sim_transport_delay;
    vessel.tail_grams = opt.sim_tail;
    vessel.leak_gps = opt.leak_gps;
    vessel.leak_after = opt.leak_after;
    if (opt.cylinder_radius_mm > 0.0) {
        vessel.cylinder = CylinderOnset(opt.cylinder_radius_mm, opt.cylinder_height_mm);
        vessel.contents_grams = opt.contents_grams;
    }

    ScaleFilter filter;
    ScaleFilter::Config fc;
    fc.max_step_grams = 3.0 * 10.0 * kScaleDt;
    fc.rate_window_sec = 3.0 * kScaleDt;
    fc.history = 8;
    filter.configure(fc);

    double now = 0.0;
    // The tilt the planner reasons about is measured from the carried attitude,
    // so it starts at zero however the vessel is actually held; the simulated
    // vessel sees grasp_tilt added to it.
    double tilt = 0.0;
    double next_sample = 0.0;
    const double on_pan =
        std::isnan(opt.vessel_grams) ? request.container_grams : opt.vessel_grams;
    double last_reading = on_pan;
    bool published_any = false;

    planner.begin(request, now);
    RunResult result;

    while (now < opt.horizon) {
        vessel.step(now, kControlDt, tilt + opt.grasp_tilt);

        if (now >= next_sample && now < opt.scale_dies_at) {
            next_sample += kScaleDt;
            const double raw = on_pan + vessel.landed;
            const double quantised = std::round(raw / kResolution) * kResolution;
            ScaleFilter::Sample s;
            s.grams = quantised;
            s.stamp = now;
            // The indicator flags motion, and it is right only because this
            // harness asks it while the vessel is parked. Mid-flow it lies; see
            // ScaleFilterSettleLiesDuringDrips.
            s.stable = std::abs(quantised - last_reading) < 1e-9;
            last_reading = quantised;
            filter.push(s);
            published_any = true;
        }

        PourObservation obs;
        obs.now = now;
        obs.has_reading = published_any && filter.has_sample();
        obs.scale_fresh = obs.has_reading && filter.age(now) <= 0.5;
        obs.grams = filter.grams();
        obs.flow_rate = filter.flow_rate();
        obs.settled = filter.settled(now, planner.settle_hold());
        obs.tilt = tilt;
        obs.consecutive_rejects = filter.consecutive_rejects();
        obs.last_rejected_step = filter.last_rejected_step();

        const auto cmd = planner.update(obs);
        if (opt.trace_every > 0.0 && std::fmod(now, opt.trace_every) < 1.0 / 125.0) {
            std::printf("t %6.1f %-8s tilt %.3f onset %.3f flow %6.2f landed %6.2f left %6.2f\n",
                        now, cho_controller::fr5::pour::to_string(cmd.phase), tilt,
                        vessel.current_onset(), obs.flow_rate, vessel.landed,
                        vessel.contents_grams - vessel.poured_out);
        }
        if (result.phases_seen.empty() || result.phases_seen.back() != cmd.phase) {
            if (cmd.phase == PourPhase::Settle) {
                result.park_angles.push_back(tilt);
            }
            result.phases_seen.push_back(cmd.phase);
        }
        result.last_phase = cmd.phase;

        if (cmd.finished) {
            result.finished = true;
            result.success = cmd.success;
            result.message = cmd.message;
            result.delivered = vessel.landed;
            result.trim_pulses = planner.report().trim_pulses;
            result.seconds = now;
            return result;
        }

        tilt += cmd.tilt_rate * kControlDt;
        // What PouringController does to the integrated tilt: the two-sided
        // range, then the joint limit.
        tilt = std::clamp(tilt, -request.max_back_tilt, request.max_tilt);
        tilt = std::max(tilt, opt.tilt_floor);
        result.peak_tilt = std::max(result.peak_tilt, std::abs(tilt));
        result.min_tilt = std::min(result.min_tilt, tilt);
        now += kControlDt;
    }
    result.delivered = vessel.landed;
    result.seconds = now;
    return result;
}

}  // namespace pour_sim
