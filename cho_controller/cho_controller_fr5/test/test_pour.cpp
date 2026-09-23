// The pour law against a synthetic vessel. No ROS, no scale, no robot.
//
// The numbers the fixtures are built from are measurements, taken on the FR5
// cell's HS-AA with water on 2026-09-16:
//
//   5 Hz stream, 0.01 g resolution, 1.0 s to settle once the flow stops,
//   <= 0.21 g of tail at ~10 g/s, and -- the one that shapes the whole design --
//   at drip rates the indicator sat still and reported STABLE for 5.4 s with
//   material still arriving.
//
// The last one is why several tests below check that the planner never asks the
// scale whether it is done while the vessel is still tilted.
#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cho_controller_fr5/pour/material_profile.hpp"
#include "cho_controller_fr5/pour/pour_planner.hpp"
#include "cho_controller_fr5/pour/scale_filter.hpp"
#include "cho_controller_fr5/pour/shaping_law.hpp"
#include "cho_controller_fr5/pour/guards.hpp"
#include "pour_sim.hpp"

using cho_controller::fr5::pour::MaterialClass;
using cho_controller::fr5::pour::MaterialProfile;
using cho_controller::fr5::pour::PlannerConfig;
using cho_controller::fr5::pour::PourLimits;
using cho_controller::fr5::pour::PourObservation;
using cho_controller::fr5::pour::PourPhase;
using cho_controller::fr5::pour::PourPlanner;
using cho_controller::fr5::pour::PourRequest;
using cho_controller::fr5::pour::ScaleFilter;
using cho_controller::fr5::pour::ShapingConfig;
using cho_controller::fr5::pour::ShapingPourLaw;
using cho_controller::fr5::pour::pour_guard;
using pour_sim::RunOptions;
using pour_sim::RunResult;
using pour_sim::VesselSim;
using pour_sim::run_pour;

namespace {

PourLimits water_free()
{
    PourLimits l;
    l.max_flow_rate = 10.0;
    l.trim_flow_rate = 2.0;
    l.trim_pulse_sec = 0.15;
    l.transport_delay = 0.4;
    l.afterflow_grams = 0.3;
    l.settle_hold_sec = 1.2;
    l.tilt_rate = 0.5;
    l.seek_tilt_rate = 0.15;
    l.dose_quantum = 0.3;
    return l;
}

PourLimits honey_resistant()
{
    PourLimits l;
    l.max_flow_rate = 2.0;
    l.trim_flow_rate = 0.5;
    l.trim_pulse_sec = 0.4;
    l.transport_delay = 1.2;
    l.afterflow_grams = 4.0;
    l.settle_hold_sec = 3.0;
    l.tilt_rate = 0.2;
    l.seek_tilt_rate = 0.06;
    // The smallest amount a material this thick can be metered in is the string
    // it leaves behind every time the flow breaks, not something finer.
    l.dose_quantum = 3.0;
    return l;
}

PourLimits salt_free()
{
    PourLimits l;
    l.max_flow_rate = 8.0;
    l.trim_flow_rate = 1.5;
    l.trim_pulse_sec = 0.2;
    l.transport_delay = 0.3;
    l.afterflow_grams = 0.5;
    l.settle_hold_sec = 1.5;
    l.tilt_rate = 0.35;
    l.seek_tilt_rate = 0.10;
    l.dose_quantum = 0.5;
    return l;
}

PourLimits damp_powder_resistant()
{
    PourLimits l;
    l.max_flow_rate = 3.0;
    l.trim_flow_rate = 0.8;
    l.trim_pulse_sec = 0.5;
    l.transport_delay = 0.6;
    l.afterflow_grams = 3.0;
    l.settle_hold_sec = 3.0;
    l.tilt_rate = 0.18;
    l.seek_tilt_rate = 0.05;
    l.dose_quantum = 2.0;
    return l;
}

MaterialProfile liquid_profile()
{
    MaterialProfile p;
    p.free = water_free();
    p.resistant = honey_resistant();
    return p;
}

MaterialProfile granular_profile()
{
    MaterialProfile p;
    p.free = salt_free();
    p.resistant = damp_powder_resistant();
    return p;
}


PourPlanner make_planner(PlannerConfig config = PlannerConfig{})
{
    PourPlanner planner;
    std::string why;
    EXPECT_TRUE(planner.configure(config, liquid_profile(), granular_profile(), why)) << why;
    return planner;
}

PourRequest water_request(double target = 50.0, double container = 139.15)
{
    PourRequest r;
    r.target_grams = target;
    r.container_grams = container;
    r.tolerance = 0.5;
    r.timeout = 180.0;
    r.max_tilt = 1.2;
    r.max_tilt_rate = 0.0;
    r.material = MaterialClass::Liquid;
    r.flow_index = 0.0;
    return r;
}

}  // namespace

// ---------------------------------------------------------------- profiles --

TEST(MaterialProfile, InterpolatesBetweenEndpointsAndClampsOutside)
{
    const auto profile = liquid_profile();
    EXPECT_DOUBLE_EQ(profile.at(0.0).max_flow_rate, water_free().max_flow_rate);
    EXPECT_DOUBLE_EQ(profile.at(1.0).max_flow_rate, honey_resistant().max_flow_rate);
    EXPECT_NEAR(profile.at(0.5).max_flow_rate, 6.0, 1e-9);
    // Past the endpoints nobody characterised the material, so the guess stops
    // growing rather than extrapolating into a tilt rate no one has seen.
    EXPECT_DOUBLE_EQ(profile.at(5.0).max_flow_rate, honey_resistant().max_flow_rate);
    EXPECT_DOUBLE_EQ(profile.at(-3.0).max_flow_rate, water_free().max_flow_rate);
}

TEST(MaterialProfile, NonFiniteIndexFallsBackToFreeRatherThanPropagating)
{
    const auto profile = liquid_profile();
    const auto limits = profile.at(std::nan(""));
    EXPECT_TRUE(std::isfinite(limits.tilt_rate));
    EXPECT_DOUBLE_EQ(limits.max_flow_rate, water_free().max_flow_rate);
}

TEST(MaterialProfile, RejectsATransposedPair)
{
    MaterialProfile swapped;
    swapped.free = honey_resistant();
    swapped.resistant = water_free();
    std::string why;
    EXPECT_FALSE(swapped.validate("liquid", why));
    EXPECT_NE(why.find("transposed"), std::string::npos) << why;
}

TEST(MaterialProfile, RejectsATrimRateFasterThanTheBulkRate)
{
    MaterialProfile p = liquid_profile();
    p.free.trim_flow_rate = p.free.max_flow_rate * 2.0;
    std::string why;
    EXPECT_FALSE(p.validate("liquid", why));
    EXPECT_NE(why.find("SLOWER"), std::string::npos) << why;
}

// ------------------------------------------------------------ scale filter --

TEST(ScaleFilter, RejectsAStepNoFlowCouldProduce)
{
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);

    EXPECT_TRUE(filter.push({100.0, 0.0, true}));
    EXPECT_TRUE(filter.push({102.0, 0.2, false}));
    // The pan was knocked. 90 g in 200 ms is not a pour at any rate.
    EXPECT_FALSE(filter.push({192.0, 0.4, false}));
    EXPECT_EQ(filter.consecutive_rejects(), 1);
    // Nothing moved: the rejected sample did not become the new mass.
    EXPECT_DOUBLE_EQ(filter.grams(), 102.0);
    EXPECT_TRUE(filter.push({104.0, 0.6, false}));
    EXPECT_EQ(filter.consecutive_rejects(), 0);
}

TEST(ScaleFilter, RejectsNaNAndNonAdvancingStamps)
{
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);

    EXPECT_TRUE(filter.push({100.0, 0.0, true}));
    EXPECT_FALSE(filter.push({std::nan(""), 0.2, true}));
    EXPECT_FALSE(filter.push({101.0, 0.0, true}));   // repeated stamp
    EXPECT_FALSE(filter.push({101.0, -0.1, true}));  // reordered
    EXPECT_TRUE(std::isfinite(filter.grams()));
    EXPECT_TRUE(std::isfinite(filter.flow_rate()));
}

TEST(ScaleFilter, FitsAFlowRateFromA5HzRamp)
{
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    c.rate_window_sec = 0.6;
    filter.configure(c);

    // 9 g/s, quantised to 0.01 g the way the HS-AA reports it.
    for (int i = 0; i < 12; ++i) {
        const double t = 0.2 * i;
        const double grams = std::round((100.0 + 9.0 * t) / 0.01) * 0.01;
        filter.push({grams, t, false});
    }
    EXPECT_NEAR(filter.flow_rate(), 9.0, 0.2);
}

TEST(ScaleFilter, SettleNeedsBothTheStableFlagAndAQuietValue)
{
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);

    filter.push({100.0, 0.0, true});
    filter.push({100.0, 0.2, true});
    EXPECT_FALSE(filter.settled(0.2, 1.2));   // quiet, but not for long enough
    filter.push({100.0, 1.4, true});
    EXPECT_TRUE(filter.settled(1.4, 1.2));
    // One flagged-unstable sample is enough to withdraw it, even unchanged.
    filter.push({100.0, 1.6, false});
    EXPECT_FALSE(filter.settled(1.6, 1.2));
}

TEST(ScaleFilterSettleLiesDuringDrips, DocumentedBlindSpot)
{
    // Reproduces the measurement that shaped the design: during a 1.5 g/s drip
    // the HS-AA held one value and flagged it stable for 5.4 s while material
    // was still arriving. settled() answers "yes" here and it is WRONG -- which
    // is exactly why PourPlanner only asks once the vessel is parked below the
    // onset angle, where no flow is possible by construction.
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);

    for (int i = 0; i <= 27; ++i) {
        filter.push({358.13, 0.2 * i, true});
    }
    EXPECT_TRUE(filter.settled(5.4, 1.2));
    EXPECT_TRUE(filter.settled(5.4, 3.0));
    // No hold time rescues it: the gap between avalanches has no upper bound.
    EXPECT_TRUE(filter.settled(5.4, 5.0));
}

// ------------------------------------------------------------------ pours --

TEST(PourPlanner, RefusesAVesselThatIsNotTheOneDescribed)
{
    // The goal describes a 200 g vessel; the pan holds the 139.15 g flask. That
    // is the case an offset-only design pours anyway -- and it is not
    // hypothetical: on the rig the same flask read 139.15 g dry and 149.02 g
    // with the last pour's water still in it.
    auto planner = make_planner();
    PourRequest mismatched = water_request(50.0);
    mismatched.container_grams = 200.0;

    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);
    planner.begin(mismatched, 0.0);

    bool finished = false;
    std::string message;
    for (int i = 0; i < 200 && !finished; ++i) {
        const double t = 0.2 * i;
        filter.push({139.15, t, true});

        PourObservation obs;
        obs.now = t;
        obs.has_reading = true;
        obs.scale_fresh = true;
        obs.grams = 139.15;
        obs.flow_rate = 0.0;
        obs.settled = filter.settled(t, planner.settle_hold());
        obs.tilt = 0.0;

        const auto cmd = planner.update(obs);
        finished = cmd.finished;
        message = cmd.message;
        EXPECT_DOUBLE_EQ(cmd.tilt_rate, 0.0) << "nothing may tilt before the vessel checks out";
        if (finished) {
            EXPECT_FALSE(cmd.success);
        }
    }
    ASSERT_TRUE(finished) << "a mismatched vessel must end the goal, not wait forever";
    EXPECT_NE(message.find("vessel"), std::string::npos) << message;
}

TEST(PourPlanner, PoursWaterToTargetWithinTolerance)
{
    auto planner = make_planner();
    const auto result = run_pour(planner, water_request(50.0), RunOptions{});
    ASSERT_TRUE(result.finished) << "planner never terminated";
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_NEAR(result.delivered, 50.0, 1.0);
}

TEST(PourPlanner, NeverJudgesDoneWhileTheVesselIsStillTilted)
{
    // The phase order is the safety property: every Settle is reached through a
    // Retract, never straight from Bulk or Trim.
    auto planner = make_planner();
    const auto result = run_pour(planner, water_request(50.0), RunOptions{});
    ASSERT_TRUE(result.finished);
    for (std::size_t i = 0; i < result.phases_seen.size(); ++i) {
        if (result.phases_seen[i] == PourPhase::Settle) {
            ASSERT_GT(i, 0u);
            EXPECT_EQ(result.phases_seen[i - 1], PourPhase::Retract)
                << "a settle was entered without parking the vessel first";
        }
    }
}

TEST(PourPlanner, HandlesAVesselGrippedOffSquare)
{
    // The beaker is handed over already leaning 0.10 rad toward the spout, so it
    // reaches its onset that much earlier. Nothing is configured for it: the
    // onset is found, and the park angle is allowed below the carried attitude.
    auto planner = make_planner();
    RunOptions opt;
    opt.grasp_tilt = 0.10;
    const auto result = run_pour(planner, water_request(50.0), opt);
    ASSERT_TRUE(result.finished) << "planner never terminated";
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_NEAR(result.delivered, 50.0, 1.0);
}

TEST(PourPlanner, ATiltedGraspReachesItsOnsetSooner)
{
    auto square = make_planner();
    auto tilted = make_planner();
    RunOptions opt_tilted;
    opt_tilted.grasp_tilt = 0.10;

    const auto a = run_pour(square, water_request(50.0), RunOptions{});
    const auto b = run_pour(tilted, water_request(50.0), opt_tilted);
    ASSERT_TRUE(a.success) << a.message;
    ASSERT_TRUE(b.success) << b.message;
    EXPECT_LT(b.peak_tilt, a.peak_tilt)
        << "a vessel already leaning should not have to be tilted as far again";
}

TEST(PourPlanner, ThickLiquidStillLandsAndTakesLonger)
{
    auto thin = make_planner();
    auto thick = make_planner();

    PourRequest syrup = water_request(50.0);
    syrup.flow_index = 1.0;
    syrup.tolerance = 1.0;  // below the profile's quantum; the floor should win

    RunOptions thick_opt;
    thick_opt.sim_gain = 8.0;            // pours far less readily
    thick_opt.sim_transport_delay = 1.1; // and takes longer to arrive
    thick_opt.sim_tail = 3.0;            // with a much larger tail

    const auto fast = run_pour(thin, water_request(50.0), RunOptions{});
    const auto slow = run_pour(thick, syrup, thick_opt);
    ASSERT_TRUE(slow.finished) << "thick pour never terminated";
    EXPECT_TRUE(slow.success) << slow.message;
    EXPECT_NEAR(slow.delivered, 50.0, 3.0);
    EXPECT_GT(slow.seconds, fast.seconds);
}

TEST(PourPlanner, GranularMediaLandsWithinItsOwnDoseQuantum)
{
    auto planner = make_planner();
    PourRequest sugar = water_request(40.0);
    sugar.material = MaterialClass::Granular;
    sugar.flow_index = 0.0;
    sugar.tolerance = 0.5;  // below the profile's quantum; the floor should win

    RunOptions opt;
    opt.sim_gain = 30.0;
    opt.sim_transport_delay = 0.3;
    opt.sim_tail = 0.5;

    const auto result = run_pour(planner, sugar, opt);
    ASSERT_TRUE(result.finished) << "planner never terminated";
    EXPECT_TRUE(result.success) << result.message;
    // Salt's configured quantum is 0.5 g, and the pour is allowed a couple of
    // them: the floor is what one avalanche weighs, not what the goal asked for.
    EXPECT_NEAR(result.delivered, 40.0, 1.5);
}

TEST(PourPlanner, TheDoseQuantumFloorsTheRequestedTolerance)
{
    auto planner = make_planner();
    PourRequest powder = water_request(40.0);
    powder.material = MaterialClass::Granular;
    powder.flow_index = 1.0;   // dose_quantum 2.0 g
    powder.tolerance = 0.05;   // not achievable by any law

    planner.begin(powder, 0.0);
    EXPECT_DOUBLE_EQ(planner.report().effective_tolerance, 2.0);
}

TEST(PourPlanner, ReportsOvershootInsteadOfChasingIt)
{
    auto planner = make_planner();
    PourRequest request = water_request(50.0);
    request.tolerance = 0.2;

    RunOptions opt;
    // The vessel pours at a rate the profile knows, but when the stream breaks
    // it keeps draining twenty times longer than water does -- the surprise a
    // flow_index of 0 cannot anticipate. The bulk phase stops in the right
    // place and the tail carries it past anyway.
    opt.sim_gain = 40.0;
    opt.sim_transport_delay = 0.4;
    opt.sim_tail = 6.0;

    const auto result = run_pour(planner, request, opt);
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("overpoured"), std::string::npos) << result.message;
    // And it stopped: the old law drove the wrist backwards through the carried
    // attitude on a negative error until it hit the tilt bound.
    EXPECT_LE(result.peak_tilt, request.max_tilt + 1e-6);
}

TEST(PourPlanner, ParksTheVesselWhenTheScaleGoesQuiet)
{
    auto planner = make_planner();
    RunOptions opt;
    opt.scale_dies_at = 6.0;

    const auto result = run_pour(planner, water_request(200.0), opt);
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("quiet"), std::string::npos) << result.message;
    EXPECT_EQ(result.last_phase, PourPhase::Done);
}

TEST(PourPlanner, GivesUpWhenTheTiltBoundProducesNoFlow)
{
    auto planner = make_planner();
    PourRequest request = water_request(50.0);
    request.max_tilt = 0.05;  // well below the vessel's 0.15 rad onset

    const auto result = run_pour(planner, request, RunOptions{});
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("nothing came out"), std::string::npos) << result.message;
}

TEST(PourPlanner, CancelParksTheVesselBeforeEnding)
{
    auto planner = make_planner();
    PourRequest request = water_request(500.0);

    VesselSim vessel;
    ScaleFilter filter;
    ScaleFilter::Config c;
    c.max_step_grams = 6.0;
    filter.configure(c);

    double now = 0.0, tilt = 0.0, next_sample = 0.0, last = request.container_grams;
    planner.begin(request, now);
    bool cancelled = false, finished = false;
    double tilt_at_cancel = 0.0;

    for (int i = 0; i < 125 * 120 && !finished; ++i) {
        vessel.step(now, 1.0 / 125.0, tilt);
        if (now >= next_sample) {
            next_sample += 0.2;
            const double g = std::round((request.container_grams + vessel.landed) / 0.01) * 0.01;
            filter.push({g, now, std::abs(g - last) < 1e-9});
            last = g;
        }
        PourObservation obs;
        obs.now = now;
        obs.has_reading = filter.has_sample();
        obs.scale_fresh = obs.has_reading && filter.age(now) <= 0.5;
        obs.grams = filter.grams();
        obs.flow_rate = filter.flow_rate();
        obs.settled = filter.settled(now, planner.settle_hold());
        obs.tilt = tilt;

        if (!cancelled && vessel.landed > 20.0) {
            planner.cancel();
            cancelled = true;
            tilt_at_cancel = tilt;
        }
        const auto cmd = planner.update(obs);
        finished = cmd.finished;
        if (finished) {
            EXPECT_FALSE(cmd.success);
            EXPECT_EQ(cmd.message, "cancelled");
        }
        tilt += cmd.tilt_rate / 125.0;
        now += 1.0 / 125.0;
    }
    ASSERT_TRUE(finished);
    EXPECT_GT(tilt_at_cancel, 0.0);
    // Back to the attitude it was carried in, not abandoned mid-tip.
    EXPECT_NEAR(tilt, 0.0, 0.01);
}

TEST(PourPlanner, ABrokenScaleStopsTheTiltAndSaysItJumpedRatherThanWentQuiet)
{
    // Still publishing, every sample rejected as impossible. Staleness is
    // judged from ACCEPTED samples, so this stops the pour exactly as silence
    // would -- and the report must say which it was. "The scale went quiet"
    // sends an operator to debug a driver that is working.
    auto planner = make_planner();
    planner.begin(water_request(50.0), 0.0);

    PourObservation obs;
    obs.now = 0.0;
    obs.has_reading = true;
    obs.scale_fresh = true;
    obs.grams = 139.15;
    obs.settled = true;
    obs.tilt = 0.0;
    planner.update(obs);   // verify passes

    obs.now = 1.0;
    obs.settled = false;
    obs.scale_fresh = false;
    obs.consecutive_rejects = 3;
    obs.last_rejected_step = 90.0;
    const auto cmd = planner.update(obs);
    EXPECT_DOUBLE_EQ(cmd.tilt_rate, 0.0);
    ASSERT_EQ(planner.phase(), PourPhase::Retract);

    obs.now = 1.1;
    const auto done = planner.update(obs);   // already at tilt 0: retract completes
    ASSERT_TRUE(done.finished);
    EXPECT_NE(done.message.find("jumped by 90"), std::string::npos) << done.message;
    EXPECT_EQ(done.message.find("quiet"), std::string::npos) << done.message;
}

// ------------------------------------------------- the back of the range --

TEST(PourPlanner, AParkThatNeverSettlesWalksDownLinearlyAndStopsAtTheBackBound)
{
    // Something other than the lip keeps adding mass to the pan, so no park
    // ever settles and the planner keeps concluding its park is too high. That
    // is the case that used to compound the margin -- 0.06, 0.12, 0.36, 1.44,
    // 7.2 rad -- and swing the wrist backwards to its joint limit.
    auto planner = make_planner();
    RunOptions opt;
    opt.leak_gps = 0.5;
    opt.leak_after = 5.0;
    opt.max_back_tilt = 0.3;

    const auto result = run_pour(planner, water_request(50.0), opt);
    ASSERT_TRUE(result.finished) << "a park that cannot settle must end the goal";
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("max_back_tilt"), std::string::npos) << result.message;
    EXPECT_GE(result.min_tilt, -opt.max_back_tilt - 1e-9)
        << "the vessel went further behind its carried attitude than allowed";

    ASSERT_GE(result.park_angles.size(), 2u);
    for (std::size_t i = 1; i < result.park_angles.size(); ++i) {
        const double gap = result.park_angles[i - 1] - result.park_angles[i];
        EXPECT_GT(gap, 0.0) << "each re-park must be lower than the last";
        // Linear: the n-th retry drops n base margins, and the base is 0.06 rad
        // for water. A compounding margin blows through this by the third.
        EXPECT_LE(gap, 0.06 * static_cast<double>(i) + 0.01) << "re-park " << i;
    }
}

TEST(PourPlanner, AZeroBackBoundNeverTurnsTheVesselBehindItsCarriedAttitude)
{
    auto planner = make_planner();
    RunOptions opt;
    opt.leak_gps = 0.5;
    opt.leak_after = 5.0;
    opt.max_back_tilt = 0.0;

    const auto result = run_pour(planner, water_request(50.0), opt);
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_GE(result.min_tilt, -1e-9);
}

TEST(PourPlanner, AParkBehindAJointLimitEndsTheGoalInsteadOfHoldingItOpen)
{
    // The arm cannot go below 0.14 rad, which is just short of where the
    // planner wants to park (onset 0.15 less the margin). A retract ignores
    // cancel, timeout and staleness -- it is what they would have asked for --
    // so without a deadline this goal would never finish.
    auto planner = make_planner();
    RunOptions opt;
    opt.tilt_floor = 0.14;
    opt.horizon = 120.0;

    const auto result = run_pour(planner, water_request(50.0), opt);
    ASSERT_TRUE(result.finished) << "the planner is still in " << to_string(result.last_phase)
                                 << " after " << result.seconds << " s";
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("could not reach"), std::string::npos) << result.message;
    EXPECT_LT(result.seconds, 60.0);
}

TEST(PourPlanner, AnUnreachableParkAfterACancelStillEnds)
{
    // Cancel asks for a retract to the carried attitude. If that, too, is
    // pinned, the deadline is still what ends the goal.
    auto planner = make_planner();
    PourRequest request = water_request(50.0);
    request.max_back_tilt = 0.3;
    planner.begin(request, 0.0);

    PourObservation obs;
    obs.has_reading = true;
    obs.scale_fresh = true;
    obs.grams = request.container_grams;
    obs.settled = true;
    obs.tilt = 0.0;
    planner.update(obs);                    // verify passes

    obs.now = 0.1;
    obs.settled = false;
    obs.tilt = 0.4;                         // mid-pour
    planner.cancel();
    planner.update(obs);
    ASSERT_EQ(planner.phase(), PourPhase::Retract);

    bool finished = false;
    std::string message;
    for (int i = 1; i < 125 * 30 && !finished; ++i) {
        obs.now = 0.1 + i / 125.0;
        obs.tilt = 0.4;                     // never moves
        const auto cmd = planner.update(obs);
        finished = cmd.finished;
        message = cmd.message;
    }
    ASSERT_TRUE(finished);
    EXPECT_NE(message.find("could not reach"), std::string::npos) << message;
    EXPECT_NE(message.find("cancelled"), std::string::npos)
        << "the original reason must survive into the report: " << message;
}

// ------------------------------------------------------------- the guard --

TEST(PourGuard, SilenceAndImpossibleReadingsAreReportedDifferently)
{
    PourObservation obs;
    obs.now = 5.0;
    obs.has_reading = true;
    obs.scale_fresh = false;

    const std::string silent = pour_guard(obs, PourPhase::Bulk, false, 0.0, 60.0);
    EXPECT_NE(silent.find("quiet"), std::string::npos) << silent;

    obs.consecutive_rejects = 2;
    obs.last_rejected_step = -139.1;
    const std::string jumped = pour_guard(obs, PourPhase::Bulk, false, 0.0, 60.0);
    EXPECT_NE(jumped.find("jumped by -139.1"), std::string::npos) << jumped;
}

TEST(PourGuard, RetractIsNeverInterruptedAndVerifyRunsItsOwnDeadline)
{
    PourObservation obs;
    obs.now = 500.0;
    obs.scale_fresh = false;
    // Every reason to stop at once, and none of them applies to a retract:
    // a retract is what each of them would have asked for.
    EXPECT_EQ(pour_guard(obs, PourPhase::Retract, true, 0.0, 60.0), "");
    // Verify has not tilted; staleness there is the container gate's business.
    EXPECT_EQ(pour_guard(obs, PourPhase::Verify, false, 0.0, 0.0), "");
    EXPECT_EQ(pour_guard(obs, PourPhase::Verify, true, 0.0, 0.0), "cancelled");
}

// ------------------------------------------------------- the shaping law --

namespace {

ShapingConfig tuned_shaping()
{
    // The gains controllers.yaml ships for the shaping law: the ones a sweep at
    // 50 g against pour_sim.hpp settled on. See the yaml for why not 0.008.
    ShapingConfig c;
    c.kp = 0.0001;
    c.kd = 0.005;
    return c;
}

ShapingPourLaw make_shaping(const ShapingConfig & config = tuned_shaping())
{
    ShapingPourLaw law;
    std::string why;
    EXPECT_TRUE(law.configure(config, why)) << why;
    return law;
}

}  // namespace

TEST(ShapingPourLaw, TheKernelIsADecayingSinusoidThatReversesSign)
{
    // The oscillation is the whole point of the paper's law: the negative lobe
    // is what makes the pour pause and pull back the way a chemist's does.
    auto law = make_shaping();
    const auto & k = law.kernel();
    ASSERT_EQ(k.size(), static_cast<std::size_t>(2.5 / 0.04));
    EXPECT_DOUBLE_EQ(k[0], 0.0);
    EXPECT_GT(*std::max_element(k.begin(), k.end()), 0.0);
    EXPECT_LT(*std::min_element(k.begin(), k.end()), 0.0);
}

TEST(ShapingPourLaw, PoursWaterToTargetAtItsTunedGains)
{
    // Slow by nature -- ~230 s on this vessel, against the phase machine's ~14 --
    // so it gets a timeout that measures accuracy rather than patience.
    auto law = make_shaping();
    PourRequest request = water_request(50.0);
    request.timeout = 400.0;
    RunOptions opt;
    opt.horizon = 500.0;
    const auto result = run_pour(law, request, opt);
    ASSERT_TRUE(result.finished) << "shaping law never terminated";
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_NEAR(result.delivered, 50.0, 1.5);
}

TEST(ShapingPourLaw, StopsAtOrPastTheTargetInsteadOfChasingAnOvershoot)
{
    // A band test (|error| < tolerance) is jumped clean over when one sample
    // carries more than twice the tolerance, and the law then sees a negative
    // error and turns the wrist backwards after grams that cannot come back.
    // The shipped gains pour fast enough to do exactly that.
    //
    // At kp 0.002 against this vessel the band version ran to the 120 s timeout,
    // swinging the wrist back to the -0.3 rad bound; stopped at or past the
    // target it ends in ~12 s having never tilted behind upright.
    ShapingConfig eager = tuned_shaping();
    eager.kp = 0.002;
    eager.kd = 0.002;
    auto law = make_shaping(eager);
    PourRequest request = water_request(50.0);
    request.tolerance = 0.2;
    request.timeout = 120.0;

    RunOptions opt;
    opt.horizon = 200.0;
    const auto result = run_pour(law, request, opt);
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("overpoured"), std::string::npos) << result.message;
    // Never swung behind the attitude it was carried in -- beyond the one
    // control step a retract may land past zero inside its epsilon. The band
    // version reached the -0.3 rad bound.
    EXPECT_GE(result.min_tilt, -0.005);
    EXPECT_LT(result.seconds, 60.0) << "an overshoot must end the goal, not run to the timeout";
}

TEST(ShapingPourLaw, SharesThePhaseMachinesGuards)
{
    // Same dead scale, same stop, same words -- the A/B would otherwise be
    // comparing two different safety envelopes.
    auto law = make_shaping();
    auto planner = make_planner();
    RunOptions opt;
    opt.scale_dies_at = 6.0;

    const auto a = run_pour(law, water_request(200.0), opt);
    const auto b = run_pour(planner, water_request(200.0), opt);
    ASSERT_TRUE(a.finished);
    ASSERT_TRUE(b.finished);
    EXPECT_FALSE(a.success);
    EXPECT_NE(a.message.find("quiet"), std::string::npos) << a.message;
    EXPECT_NE(b.message.find("quiet"), std::string::npos) << b.message;
}

TEST(ShapingPourLaw, RefusesTheVesselTheGoalDidNotDescribe)
{
    // The container gate is shared too: the shaping law must not pour into a
    // vessel the phase machine would have refused.
    auto law = make_shaping();
    PourRequest request = water_request(50.0);
    request.container_grams = 200.0;
    RunOptions opt;
    opt.vessel_grams = 139.15;
    const auto result = run_pour(law, request, opt);
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("vessel"), std::string::npos) << result.message;
    EXPECT_DOUBLE_EQ(result.peak_tilt, 0.0);
}

TEST(ShapingPourLaw, TheGainsThisControllerShippedWithOutrunTheScaleAt50g)
{
    // controllers.yaml says why the shaping law does not use 0.008 / 0.002: the
    // kernel's DC gain is ~1.25, so a 50 g error asks for the full tilt rate,
    // and the flow outruns anything the outlier bound accepts. Pinned here so
    // that the claim in the yaml stays true, and so that the pour is seen to
    // stop -- and to say why -- rather than keep tilting.
    ShapingConfig shipped;
    shipped.kp = 0.008;
    shipped.kd = 0.002;
    auto law = make_shaping(shipped);
    const auto result = run_pour(law, water_request(50.0), RunOptions{});
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("jumped"), std::string::npos) << result.message;
}

// ------------------------------------------- a vessel whose onset moves --

namespace {

// The FR5 bench's 100 mL beaker: 50 mm bore, 70 mm to the rim, filled to the
// 100 mL mark. Its onset is 0.65 rad full and 0.94 rad after 30 g has gone.
RunOptions hundred_ml_beaker()
{
    RunOptions o;
    o.cylinder_radius_mm = 25.0;
    o.cylinder_height_mm = 70.0;
    o.contents_grams = 100.0;
    o.horizon = 200.0;
    return o;
}

}  // namespace

TEST(PourPlanner, FinishesAPourFromABeakerWhoseOnsetRisesAsItEmpties)
{
    // Trim pulses used to aim at the onset the seek found. Over a 30 g pour
    // this beaker's onset rises 0.29 rad, so every pulse aimed at the old one
    // poured nothing: 8 pulses, 2.74 g short. They now creep up to wherever the
    // flow actually starts.
    for (double target : {30.0, 50.0}) {
        auto planner = make_planner();
        const auto result = run_pour(planner, water_request(target), hundred_ml_beaker());
        ASSERT_TRUE(result.finished) << target << " g never terminated";
        EXPECT_TRUE(result.success) << target << " g: " << result.message;
        EXPECT_NEAR(result.delivered, target, 0.5) << target << " g";
        EXPECT_LE(result.trim_pulses, 4) << target << " g";
    }
}

TEST(PourPlanner, EndsCleanlyWhenWhatIsLeftCannotReachTheLip)
{
    // max_tilt 1.2 rad cannot pour this beaker below ~33 g. Asking for 80 g of
    // its 100 must end, short, and say so -- not creep at the bound forever.
    auto planner = make_planner();
    const auto result = run_pour(planner, water_request(80.0), hundred_ml_beaker());
    ASSERT_TRUE(result.finished);
    EXPECT_FALSE(result.success);
    EXPECT_LT(result.delivered, 80.0);
    EXPECT_NE(result.message.find("tilt bound"), std::string::npos) << result.message;
    // It used to wait out the 180 s goal timeout while the last grams trickled.
    EXPECT_LT(result.seconds, 60.0);
}
