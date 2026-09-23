#pragma once

#include <string>

#include "cho_controller_fr5/pour/pour_types.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

struct ContainerVerdict {
    //: The gate has reached an answer. Until then the caller holds still.
    bool decided{false};
    //: The vessel on the pan is the one the goal described.
    bool ok{false};
    //: The baseline to measure the pour from, valid only when ok.
    double baseline{0.0};
    //: Why it was refused, when it was.
    std::string message;
};

/**
 * Whether the vessel on the pan is the one the goal described.
 *
 * Shared by every pour law, and deliberately so: it runs BEFORE the first tilt,
 * it is the same question whichever law pours afterwards, and having both laws
 * establish their baseline the same way is what makes a comparison between them
 * a comparison of the laws rather than of their setup.
 *
 * The check exists because the indicator is read-only -- it cannot be tared over
 * RS232 -- so the declared container weight is the pour's only zero. An offset
 * with no check pours into a vessel that is missing, is the wrong one, still
 * holds what an earlier pour left, or sits on a pan whose front-panel zero
 * someone moved. On the FR5 cell the same flask read 139.15 g dry and 149.02 g
 * with the last pour's water still in it.
 *
 * The baseline it returns is the MEASUREMENT, not the declaration it was just
 * checked against: the declaration is what makes the reading trustworthy, the
 * reading is what is accurate.
 */
ContainerVerdict verify_container(const PourObservation & obs, double container_grams,
                                  double tolerance, double started_at, double timeout);

} // namespace pour
} // namespace fr5
} // namespace cho_controller
