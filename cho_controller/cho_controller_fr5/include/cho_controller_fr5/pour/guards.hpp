#pragma once

#include <string>

#include "cho_controller_fr5/pour/pour_types.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * The obligations every pour law shares, checked before the law's own phase
 * logic. Returns why the pour must stop, or an empty string when it may go on.
 *
 * One function, called by every law, because these are the controller's
 * obligations rather than any law's: a comparison in which one law may keep
 * tilting at a dead scale is not a comparison, and a fix applied to one copy of
 * these checks and not the other would make it one silently.
 *
 * In order:
 *   - cancel;
 *   - the goal's timeout;
 *   - staleness of the newest ACCEPTED sample. Verify is exempt -- it has not
 *     tilted, and runs its own deadline -- and so is Retract, which is already
 *     what every one of these would ask for.
 *
 * There is no separate "too many rejected samples" check. A scale publishing
 * impossible values stops the pour through staleness, because rejected samples
 * are never accepted; the reject count only chooses the message. A separate
 * threshold counted in samples could never fire first anyway -- at 5 Hz, any
 * count above two outlasts a 0.5 s scale_timeout.
 */
std::string pour_guard(const PourObservation & obs, PourPhase phase, bool cancel_requested,
                       double started_at, double timeout);

/**
 * Whether the tilt is at the goal's bound, to within `epsilon`.
 *
 * Not `tilt >= max_tilt`. A law at the bound that eases back by a hair --
 * any angle law does, as its target drifts -- sits a few microradians under it,
 * and an exact comparison then resets every stall timer the instant it starts.
 * That held a 100 mL beaker asked for more than it could give at the bound, with
 * no flow, for the whole 180 s goal timeout.
 */
inline bool at_tilt_bound(const PourObservation & obs, double max_tilt, double epsilon)
{
    return obs.tilt >= max_tilt - epsilon;
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
