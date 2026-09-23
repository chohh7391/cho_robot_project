#pragma once

#include <string>

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * What is being poured. The two classes do not share a flow law.
 *
 * A liquid's outflow is a continuous monotone function of tilt and remaining
 * volume, so a flow rate can be regulated. A granular medium arrives in
 * avalanches whose size is set by the aperture and the packing: below some tilt
 * nothing moves, above it a lump goes. Regulating its flow rate is not possible
 * and the controller only meters the avalanches.
 */
enum class MaterialClass {
    Liquid = 0,
    Granular = 1,
};

/**
 * One endpoint of a class: how a material at that extreme of flow_index pours.
 *
 * Both endpoints of both classes come from configuration, never from this
 * header. Only the LIQUID/free endpoint has measured backing (water on the
 * HS-AA, 2026-09-16): 6-11 g/s continuous, 1.0 s to settle after the flow
 * stops, <=0.21 g of tail. Every other endpoint is a commissioning seed.
 */
struct PourLimits {
    //: Bulk-phase outflow ceiling [g/s]. The tilt is regulated to hold this.
    double max_flow_rate{0.0};
    //: Outflow a trim pulse is metered at [g/s]. Low on purpose: the stop error
    //: is proportional to the flow rate at the moment the tilt stops, so the
    //: last grams have to arrive slowly however fast the bulk phase ran.
    double trim_flow_rate{0.0};
    //: Shortest pulse that still breaks the material loose [s]. With
    //: trim_flow_rate this sets the smallest dose the controller can meter.
    double trim_pulse_sec{0.0};
    //: Seconds between material leaving the lip and the scale reporting it.
    //: Its uncertainty, times the flow rate, dominates the stop error.
    double transport_delay{0.0};
    //: Prior for what still arrives after the tilt stops [g]. Re-measured from
    //: the pour itself and replaced for every trim pulse after the first.
    double afterflow_grams{0.0};
    //: How long the reading must hold still, with the indicator reporting
    //: stable, before it is believed [s].
    double settle_hold_sec{0.0};
    //: Tilt rate for the bulk and trim phases [rad/s].
    double tilt_rate{0.0};
    //: Tilt rate while hunting for the angle at which flow starts [rad/s].
    //: Slower than tilt_rate: overshooting the onset of a thick or cohesive
    //: material dumps it.
    double seek_tilt_rate{0.0};
    //: Smallest amount that can arrive as one piece [g]. A tolerance below this
    //: is not achievable by any control law -- for water broken into drops it is
    //: the drop, measured at 1.2 g on the test rig -- so it floors the goal's
    //: tolerance instead of being chased.
    double dose_quantum{0.0};
};

/**
 * The two endpoints of one material class, and the interpolation between them.
 *
 * flow_index is the single number a caller supplies to say how reluctantly its
 * material flows within its class, and this is what turns that number into
 * every rate, delay and bound at once. One index moves the whole profile
 * together -- a higher one pours slower, waits longer for the stream, expects a
 * larger tail, and holds still longer before believing the scale -- which is
 * the point: those quantities are not independent, and letting a caller set
 * them separately invites a combination no material actually has.
 */
struct MaterialProfile {
    //: flow_index = 0. Water; dry salt or sugar.
    PourLimits free;
    //: flow_index = 1. Honey or syrup; damp clumping powder.
    PourLimits resistant;

    //: Linear on every field. flow_index outside [0, 1] is clamped rather than
    //: extrapolated: past the endpoints these are guesses about a material
    //: nobody characterised, and a guess should not be allowed to keep growing.
    [[nodiscard]] PourLimits at(double flow_index) const;

    //: Every field of both endpoints must be finite and positive, and the
    //: resistant endpoint must actually be the slower one. Checked at configure
    //: so a transposed pair is refused with a vessel still on the bench rather
    //: than discovered with one in the gripper.
    [[nodiscard]] bool validate(const std::string & label, std::string & why) const;
};

//: Parse "liquid" / "granular". Returns false for anything else.
bool parse_material_class(const std::string & text, MaterialClass & out);

//: For log lines and abort messages.
const char * to_string(MaterialClass material);

} // namespace pour
} // namespace fr5
} // namespace cho_controller
