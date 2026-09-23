#include "cho_controller_fr5/pour/material_profile.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>
#include <vector>

namespace cho_controller {
namespace fr5 {
namespace pour {

namespace {
double lerp(double a, double b, double s)
{
    return a + (b - a) * s;
}
}  // namespace

PourLimits MaterialProfile::at(double flow_index) const
{
    // A non-finite index is treated as 0 rather than propagated. It would
    // otherwise reach every rate and bound below at once, and a NaN tilt rate
    // is not caught by a std::clamp downstream -- clamp returns the NaN.
    const double s = std::isfinite(flow_index) ? std::clamp(flow_index, 0.0, 1.0) : 0.0;

    PourLimits out;
    out.max_flow_rate = lerp(free.max_flow_rate, resistant.max_flow_rate, s);
    out.trim_flow_rate = lerp(free.trim_flow_rate, resistant.trim_flow_rate, s);
    out.trim_pulse_sec = lerp(free.trim_pulse_sec, resistant.trim_pulse_sec, s);
    out.transport_delay = lerp(free.transport_delay, resistant.transport_delay, s);
    out.afterflow_grams = lerp(free.afterflow_grams, resistant.afterflow_grams, s);
    out.settle_hold_sec = lerp(free.settle_hold_sec, resistant.settle_hold_sec, s);
    out.tilt_rate = lerp(free.tilt_rate, resistant.tilt_rate, s);
    out.seek_tilt_rate = lerp(free.seek_tilt_rate, resistant.seek_tilt_rate, s);
    out.dose_quantum = lerp(free.dose_quantum, resistant.dose_quantum, s);
    return out;
}

bool MaterialProfile::validate(const std::string & label, std::string & why) const
{
    const std::vector<std::pair<const char *, std::pair<double, double>>> fields = {
        {"max_flow_rate", {free.max_flow_rate, resistant.max_flow_rate}},
        {"trim_flow_rate", {free.trim_flow_rate, resistant.trim_flow_rate}},
        {"trim_pulse_sec", {free.trim_pulse_sec, resistant.trim_pulse_sec}},
        {"transport_delay", {free.transport_delay, resistant.transport_delay}},
        {"afterflow_grams", {free.afterflow_grams, resistant.afterflow_grams}},
        {"settle_hold_sec", {free.settle_hold_sec, resistant.settle_hold_sec}},
        {"tilt_rate", {free.tilt_rate, resistant.tilt_rate}},
        {"seek_tilt_rate", {free.seek_tilt_rate, resistant.seek_tilt_rate}},
        {"dose_quantum", {free.dose_quantum, resistant.dose_quantum}},
    };

    for (const auto & [name, pair] : fields) {
        for (const auto & [endpoint, value] : {std::pair<const char *, double>{"free", pair.first},
                                               std::pair<const char *, double>{"resistant", pair.second}}) {
            if (!std::isfinite(value) || value <= 0.0) {
                std::ostringstream os;
                os << label << '.' << endpoint << '.' << name
                   << " must be finite and positive (got " << value << ')';
                why = os.str();
                return false;
            }
        }
    }

    // The resistant endpoint has to be the one that pours less readily. A
    // transposed pair validates field by field and then behaves backwards --
    // flow_index 1 would pour honey faster than water -- which is the kind of
    // error that only shows up as a puddle.
    const std::vector<std::pair<const char *, std::pair<double, double>>> slower = {
        {"max_flow_rate", {free.max_flow_rate, resistant.max_flow_rate}},
        {"trim_flow_rate", {free.trim_flow_rate, resistant.trim_flow_rate}},
        {"tilt_rate", {free.tilt_rate, resistant.tilt_rate}},
        {"seek_tilt_rate", {free.seek_tilt_rate, resistant.seek_tilt_rate}},
    };
    for (const auto & [name, pair] : slower) {
        if (pair.second > pair.first) {
            std::ostringstream os;
            os << label << ".resistant." << name << " (" << pair.second
               << ") is larger than " << label << ".free." << name << " (" << pair.first
               << "); the resistant endpoint is the one that pours LESS readily, so the two "
                  "look transposed";
            why = os.str();
            return false;
        }
    }

    const std::vector<std::pair<const char *, std::pair<double, double>>> longer = {
        {"transport_delay", {free.transport_delay, resistant.transport_delay}},
        {"afterflow_grams", {free.afterflow_grams, resistant.afterflow_grams}},
        {"settle_hold_sec", {free.settle_hold_sec, resistant.settle_hold_sec}},
    };
    for (const auto & [name, pair] : longer) {
        if (pair.second < pair.first) {
            std::ostringstream os;
            os << label << ".resistant." << name << " (" << pair.second
               << ") is smaller than " << label << ".free." << name << " (" << pair.first
               << "); a more resistant material takes longer, not less, so the two look "
                  "transposed";
            why = os.str();
            return false;
        }
    }

    // trim_flow_rate is what the last grams arrive at, and the whole reason it
    // exists is that it is slower than the bulk phase.
    for (const auto & [endpoint, limits] : {std::pair<const char *, const PourLimits *>{"free", &free},
                                            std::pair<const char *, const PourLimits *>{"resistant", &resistant}}) {
        if (limits->trim_flow_rate > limits->max_flow_rate) {
            std::ostringstream os;
            os << label << '.' << endpoint << ".trim_flow_rate (" << limits->trim_flow_rate
               << ") exceeds max_flow_rate (" << limits->max_flow_rate
               << "); the trim phase exists to arrive SLOWER than the bulk phase";
            why = os.str();
            return false;
        }
    }
    return true;
}

bool parse_material_class(const std::string & text, MaterialClass & out)
{
    if (text == "liquid") {
        out = MaterialClass::Liquid;
        return true;
    }
    if (text == "granular") {
        out = MaterialClass::Granular;
        return true;
    }
    return false;
}

const char * to_string(MaterialClass material)
{
    return material == MaterialClass::Granular ? "granular" : "liquid";
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
