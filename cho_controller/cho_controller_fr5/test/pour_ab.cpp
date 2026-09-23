// Run both pour laws against the same synthetic vessels and print how they did.
//
//   ros2 run is not how this is reached -- it is a development tool, built with
//   the tests and left in the build tree:
//
//     ~/ros2_ws/build/cho_controller_fr5/pour_ab            this cell's scale
//     ~/ros2_ws/build/cho_controller_fr5/pour_ab 2.6        +2.6 s, ~ the paper's
//
// The numbers controllers.yaml quotes for the two laws come from here. They are
// a comparison on pour_sim.hpp's vessel, which the phase machine was developed
// against and the shaping law was only gain-swept on: read them as a reason to
// run both on the rig, not as the answer.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "cho_controller_fr5/pour/pour_planner.hpp"
#include "cho_controller_fr5/pour/shaping_law.hpp"
#include "pour_sim.hpp"

using namespace cho_controller::fr5::pour;
using pour_sim::RunOptions;
using pour_sim::run_pour;

int main(int argc, char ** argv)
{
    // Extra delay between the lip and the reading [s], on top of the 0.4 s
    // this cell measured. The paper's scale was ~3 s behind.
    const double extra = argc > 1 ? std::atof(argv[1]) : 0.0;

    // The profile the phase machine gets. Its transport_delay and settle hold
    // are the RIG's, so a slower scale is told to it -- the shaping law's gains
    // are the same either way, which is the point of that law.
    const double tau = 0.4 + extra;
    const double hold = extra > 0.0 ? 3.0 : 1.2;
    MaterialProfile liquid;
    liquid.free = {10.0, 2.0, 0.15, tau, 0.3, hold, 0.5, 0.15, 0.3};
    liquid.resistant = {2.0, 0.5, 0.4, std::max(1.2, 3.0 * tau), 4.0, std::max(3.0, 2.5 * hold),
                        0.2, 0.06, 3.0};

    std::string why;
    PourPlanner planner;
    if (!planner.configure(PlannerConfig{}, liquid, liquid, why)) {
        std::printf("phase machine rejected its configuration: %s\n", why.c_str());
        return 1;
    }
    ShapingConfig shaping_config;
    shaping_config.kp = 0.0001;   // controllers.yaml's shipped shaping gains
    shaping_config.kd = 0.005;
    ShapingPourLaw shaping;
    if (!shaping.configure(shaping_config, why)) {
        std::printf("shaping law rejected its configuration: %s\n", why.c_str());
        return 1;
    }

    // Grasp off-square x how readily the vessel pours x how much drains after.
    std::vector<RunOptions> cases;
    for (double grasp : {0.0, 0.05, 0.10}) {
        for (double gain : {25.0, 40.0, 70.0}) {
            for (double tail : {0.15, 0.5, 1.5}) {
                RunOptions o;
                o.grasp_tilt = grasp;
                o.sim_gain = gain;
                o.sim_tail = tail;
                o.sim_transport_delay = 0.4 + extra;
                o.horizon = 300.0;
                cases.push_back(o);
            }
        }
    }

    PourRequest request;
    request.target_grams = 50.0;
    request.container_grams = 139.15;
    request.tolerance = 0.5;
    request.timeout = 280.0;
    request.max_tilt = 1.2;

    std::printf("extra scale delay %.1f s, %zu cases, target %.0f g\n\n", extra, cases.size(),
                request.target_grams);
    std::printf("%-14s %8s %8s %8s %7s %6s %8s\n", "law", "mean|e|", "worst", "bias", "rel%",
                "ok", "mean s");
    for (PourLaw * law : {static_cast<PourLaw *>(&planner), static_cast<PourLaw *>(&shaping)}) {
        double sum = 0.0, worst = 0.0, bias = 0.0, seconds = 0.0;
        int ok = 0;
        for (const auto & c : cases) {
            const auto r = run_pour(*law, request, c);
            const double e = r.delivered - request.target_grams;
            sum += std::abs(e);
            bias += e;
            worst = std::max(worst, std::abs(e));
            seconds += r.seconds;
            ok += r.success ? 1 : 0;
        }
        const double n = static_cast<double>(cases.size());
        std::printf("%-14s %8.2f %8.2f %+8.2f %7.1f %3d/%-2zu %8.1f\n", law->name(), sum / n, worst,
                    bias / n, 100.0 * sum / n / request.target_grams, ok, cases.size(),
                    seconds / n);
    }
    return 0;
}
