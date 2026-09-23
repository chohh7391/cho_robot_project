#include "cho_controller_fr5/pour/pour_types.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

const char * to_string(PourPhase phase)
{
    switch (phase) {
        case PourPhase::Verify:  return "verify";
        case PourPhase::Seek:    return "seek";
        case PourPhase::Bulk:    return "bulk";
        case PourPhase::Retract: return "retract";
        case PourPhase::Settle:  return "settle";
        case PourPhase::Trim:    return "trim";
        case PourPhase::Done:    return "done";
    }
    return "?";
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
