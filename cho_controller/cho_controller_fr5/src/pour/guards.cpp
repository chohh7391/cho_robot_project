#include "cho_controller_fr5/pour/guards.hpp"

#include <sstream>

namespace cho_controller {
namespace fr5 {
namespace pour {

std::string pour_guard(const PourObservation & obs, PourPhase phase, bool cancel_requested,
                       double started_at, double timeout)
{
    if (phase == PourPhase::Retract || phase == PourPhase::Done) {
        return "";
    }
    if (cancel_requested) {
        return "cancelled";
    }
    if (timeout > 0.0 && (obs.now - started_at) > timeout) {
        return "timed out before reaching the target weight";
    }
    if (!obs.scale_fresh && phase != PourPhase::Verify) {
        std::ostringstream os;
        if (obs.has_reading && obs.consecutive_rejects > 0) {
            // Still publishing, but nothing it said could be a pour. Saying
            // "quiet" here sends whoever reads it to debug a driver that works.
            os << "the scale reading jumped by " << obs.last_rejected_step
               << " g between two samples and stayed there (" << obs.consecutive_rejects
               << " readings rejected). No pour moves that fast: the pan was knocked, "
                  "something was put on or taken off it, or the vessel was lifted";
        } else {
            os << "the scale went quiet mid-pour (no reading for longer than scale_timeout)";
        }
        return os.str();
    }
    return "";
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
