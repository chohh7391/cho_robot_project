#include "cho_controller_fr5/pour/container_gate.hpp"

#include <cmath>
#include <sstream>

namespace cho_controller {
namespace fr5 {
namespace pour {

ContainerVerdict verify_container(const PourObservation & obs, double container_grams,
                                  double tolerance, double started_at, double timeout)
{
    ContainerVerdict verdict;

    if (obs.has_reading && obs.scale_fresh && obs.settled) {
        const double offset = obs.grams - container_grams;
        verdict.decided = true;
        if (std::abs(offset) > tolerance) {
            std::ostringstream os;
            os << "the goal described a " << container_grams << " g vessel but the scale reads "
               << obs.grams << " g (off by " << offset
               << " g). Nothing is poured into a vessel that is not the one described: it is "
                  "missing, it is the wrong one, it still holds what an earlier pour left, or "
                  "the front-panel zero has moved";
            verdict.message = os.str();
            return verdict;
        }
        verdict.ok = true;
        verdict.baseline = obs.grams;
        return verdict;
    }

    if ((obs.now - started_at) > timeout) {
        verdict.decided = true;
        std::ostringstream os;
        if (!obs.has_reading) {
            os << "no scale reading arrived within verify_timeout of the goal";
        } else if (obs.consecutive_rejects > 0) {
            os << "the reading jumped by " << obs.last_rejected_step
               << " g after the goal started and stayed there, which no pour does: something "
                  "was put on or taken off the pan once the goal was sent. Settle the vessel on "
                  "the pan before sending the goal";
        } else if (obs.flow_rate > 0.1) {
            os << "the reading never settled and is still climbing at " << obs.flow_rate
               << " g/s: the vessel is already pouring at the attitude it was handed over in, "
                  "so there is no baseline to measure from. Park it upright before the pour";
        } else {
            os << "the reading never settled within verify_timeout; something on the pan is "
                  "still moving";
        }
        verdict.message = os.str();
        return verdict;
    }
    return verdict;
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
