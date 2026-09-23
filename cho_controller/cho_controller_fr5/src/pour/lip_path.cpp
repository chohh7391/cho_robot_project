#include "cho_controller_fr5/pour/lip_path.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace cho_controller {
namespace fr5 {
namespace pour {

namespace {
//: Millimetres to one decimal, for messages: every length in them is a few mm
//: to a few cm, and a person reading one is holding a ruler.
std::string mm(double metres)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << metres * 1000.0 << " mm";
    return os.str();
}

std::string rad(double r)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(2) << r << " rad";
    return os.str();
}

//: Every sampled point on the surface of `box`, at most `step` apart along
//: each edge. Surface only: the region a box must stay out of reaches down
//: without limit, so a box that enters it does so through its surface.
void sample_box_surface(const HandBox & box, double step, std::vector<Eigen::Vector3d> & out)
{
    const Eigen::Vector3d size = box.max - box.min;
    int n[3];
    for (int k = 0; k < 3; ++k) {
        n[k] = std::max(1, static_cast<int>(std::ceil(size(k) / step)));
    }
    for (int i = 0; i <= n[0]; ++i) {
        for (int j = 0; j <= n[1]; ++j) {
            for (int k = 0; k <= n[2]; ++k) {
                const bool surface = i == 0 || i == n[0] || j == 0 || j == n[1] ||
                                     k == 0 || k == n[2];
                if (!surface) {
                    continue;
                }
                out.emplace_back(box.min.x() + size.x() * i / n[0],
                                 box.min.y() + size.y() * j / n[1],
                                 box.min.z() + size.z() * k / n[2]);
            }
        }
    }
}
}  // namespace

bool LipPathConfig::validate(std::string & why) const
{
    const std::pair<const char *, double> positives[] = {
        {"clearance", clearance},
        {"max_height", max_height},
        {"inset", inset},
        {"gap", gap},
        {"landing_by_tilt", landing_by_tilt},
        {"max_align_distance", max_align_distance},
        {"max_axis_elevation", max_axis_elevation},
        {"max_centering_error", max_centering_error},
        {"max_tag_distance", max_tag_distance},
        {"outline_step", outline_step},
        {"hand_step", hand_step},
        {"scan_step", scan_step},
    };
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            std::ostringstream os;
            os << name << " must be finite and positive (got " << value << ')';
            why = os.str();
            return false;
        }
    }
    if (!std::isfinite(landing_margin) || landing_margin < 0.0 || landing_margin >= inset) {
        std::ostringstream os;
        os << "landing_margin must be zero or positive and less than inset (got "
           << landing_margin << " against " << inset
           << "): it is how far in counts as landing, and inset is how far in the lip is taken";
        why = os.str();
        return false;
    }
    if (max_height < clearance) {
        why = "max_height must not be below clearance: they are the two ends of the range the "
              "lip is held in above the rim";
        return false;
    }
    if (max_axis_elevation >= M_PI_2) {
        why = "max_axis_elevation must be under pi/2: a vertical pour axis pours nothing";
        return false;
    }
    // The constraint holds AT samples. A spacing wider than the gap it keeps
    // could let the stretch between two samples into the receiver.
    if (outline_step > gap || hand_step > gap) {
        std::ostringstream os;
        os << "outline_step (" << outline_step << ") and hand_step (" << hand_step
           << ") must not exceed gap (" << gap << "): the clearance is enforced at sampled "
              "points, and a wider spacing leaves room between them";
        why = os.str();
        return false;
    }
    if (!closing_axis_in_ee.allFinite() || closing_axis_in_ee.norm() < 1e-9) {
        why = "closing_axis_in_ee must be a nonzero vector: the direction the jaws close along";
        return false;
    }
    for (std::size_t i = 0; i < hand_boxes.size(); ++i) {
        const auto & box = hand_boxes[i];
        if (!box.min.allFinite() || !box.max.allFinite() ||
            (box.max - box.min).minCoeff() <= 0.0) {
            std::ostringstream os;
            os << "hand box " << i << " must have finite corners with max above min on every axis";
            why = os.str();
            return false;
        }
    }
    return true;
}

Eigen::Vector3d LipPath::to_base(double inset, double lateral, double height) const
{
    return receiver_.rim_center + x_ * (inset - receiver_.radius) + y_ * lateral +
           Eigen::Vector3d::UnitZ() * height;
}

double LipPath::inset_limit_at(double tilt, double height) const
{
    const double c = std::cos(tilt);
    const double s = std::sin(tilt);
    const double band = receiver_.radius + config_.gap;
    double limit = config_.inset;
    for (const auto & d : points_) {
        // The point, rotated about the tilt axis through the lip: forward
        // along x, and up.
        const double up = -d.x() * s + d.z() * c;
        if (height + up >= config_.gap) {
            continue;   // above the rim with room to spare: nothing to hit
        }
        const double lateral = std::abs(d.y());
        if (lateral >= band) {
            continue;   // beside the receiver, not over it
        }
        const double forward = d.x() * c + d.z() * s;
        // Near edge of the receiver, inflated by the gap, at this lateral offset.
        const double edge = receiver_.radius - std::sqrt(band * band - lateral * lateral);
        limit = std::min(limit, edge - forward);
    }
    return limit;
}

double LipPath::inset_limit(double tilt) const
{
    return inset_limit_at(tilt, height_);
}

double LipPath::required_height(double tilt) const
{
    // The lowest the lip could be held and still be `landing_margin` inside at
    // this tilt: every point that would keep it further out has to be lifted
    // clear of the rim instead.
    const double c = std::cos(tilt);
    const double s = std::sin(tilt);
    const double band = receiver_.radius + config_.gap;
    double needed = config_.clearance;
    for (const auto & d : points_) {
        const double lateral = std::abs(d.y());
        if (lateral >= band) {
            continue;
        }
        const double forward = d.x() * c + d.z() * s;
        const double edge = receiver_.radius - std::sqrt(band * band - lateral * lateral);
        if (edge - forward >= config_.landing_margin) {
            continue;
        }
        const double up = -d.x() * s + d.z() * c;
        needed = std::max(needed, config_.gap - up);
    }
    return needed;
}

Eigen::Isometry3d LipPath::vessel_pose(double tilt, double inset) const
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(tilt, y_).toRotationMatrix() * R_v0_;
    T.translation() = to_base(inset, 0.0, height_) - T.linear() * lip_in_vessel_;
    return T;
}

Eigen::Isometry3d LipPath::ee_pose(double tilt, double inset) const
{
    return vessel_pose(tilt, inset) * T_v_ee_;
}

Eigen::Isometry3d LipPath::ee_pose_aligning(double s) const
{
    Eigen::Vector3d lip = waypoints_.back();
    const double clamped = std::clamp(s, 0.0, align_length_);
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        if (clamped <= waypoint_s_[i]) {
            const double span = waypoint_s_[i] - waypoint_s_[i - 1];
            const double f = span > 0.0 ? (clamped - waypoint_s_[i - 1]) / span : 1.0;
            lip = waypoints_[i - 1] + f * (waypoints_[i] - waypoints_[i - 1]);
            break;
        }
    }
    Eigen::Isometry3d T = ee_start_;
    T.translation() += lip - lip0_;
    return T;
}

void LipPath::build_at_height(double inset0, double lateral0, double height0, double max_tilt)
{
    // Points that could be below the rim at some height the move passes
    // through. The lowest of the two ends is enough: lower is only ever worse.
    const double lowest = std::min(height0, height_);
    double furthest = -std::numeric_limits<double>::infinity();
    for (const auto & d : points_) {
        if (lowest + d.z() < config_.gap) {
            furthest = std::max(furthest, d.x());
        }
    }
    // Behind this, every one of them is outside the receiver whatever its
    // height; the rest clear the rim throughout.
    const double inset_clear = std::isfinite(furthest)
        ? -config_.gap - furthest : std::numeric_limits<double>::infinity();
    const double retreat = std::min(inset0, inset_clear);
    // Back out along x if needed, then across and to height on that line, then
    // along x to where the tilt starts. No segment can meet the receiver: the
    // first only moves away from it; the second happens where every point that
    // could be low enough to hit it is behind its rim; and the third runs on
    // the side of the constraint the tilt starts on, since whatever is low
    // enough to be constrained at the final height was already behind the
    // retreat line.
    waypoints_ = {lip0_, to_base(retreat, lateral0, height0), to_base(retreat, 0.0, height_),
                  to_base(inset_limit(0.0), 0.0, height_)};
    waypoint_s_.assign(waypoints_.size(), 0.0);
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        waypoint_s_[i] = waypoint_s_[i - 1] + (waypoints_[i] - waypoints_[i - 1]).norm();
    }
    align_length_ = waypoint_s_.back();

    // The tilts that pour into the receiver: the first contiguous run of them.
    landing_tilt_ = std::numeric_limits<double>::infinity();
    last_landing_tilt_ = -std::numeric_limits<double>::infinity();
    const int n_scan = std::max(1, static_cast<int>(std::ceil(max_tilt / config_.scan_step)));
    for (int i = 0; i <= n_scan; ++i) {
        const double tilt = max_tilt * i / n_scan;
        const bool lands = inset_limit(tilt) >= config_.landing_margin;
        if (lands && !std::isfinite(landing_tilt_)) {
            landing_tilt_ = tilt;
        }
        if (std::isfinite(landing_tilt_)) {
            if (!lands) {
                break;
            }
            last_landing_tilt_ = tilt;
        }
    }
}

bool LipPath::plan(const Eigen::Isometry3d & ee_start, const Eigen::Vector3d & pour_axis,
                   const Eigen::Vector3d & tag_position, double max_tilt,
                   const HeldVessel & vessel, const Receiver & receiver,
                   const LipPathConfig & config, std::string & why)
{
    planned_ = false;
    if (!config.validate(why)) {
        return false;
    }
    if (!(std::isfinite(vessel.radius) && vessel.radius > 0.0 && std::isfinite(vessel.height) &&
          vessel.height > 0.0 && vessel.tag_in_vessel.allFinite())) {
        why = "the held vessel needs a finite positive radius and height and a finite marker "
              "position on it";
        return false;
    }
    if (!(std::isfinite(receiver.radius) && receiver.radius > 0.0 &&
          receiver.rim_center.allFinite())) {
        why = "the receiver needs a finite rim centre and a positive radius";
        return false;
    }
    if (!ee_start.matrix().allFinite() || !pour_axis.allFinite() || !tag_position.allFinite() ||
        !std::isfinite(max_tilt) || max_tilt <= 0.0) {
        why = "non-finite EE pose, pour axis, marker position or tilt bound";
        return false;
    }
    config_ = config;
    vessel_ = vessel;
    receiver_ = receiver;
    ee_start_ = ee_start;

    // ---- the pour frame ----------------------------------------------------
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (pour_axis.norm() < 1e-9) {
        why = "the pour axis is the zero vector";
        return false;
    }
    const Eigen::Vector3d axis = pour_axis.normalized();
    const double elevation = std::asin(std::min(1.0, std::abs(axis.dot(up))));
    if (elevation > config.max_axis_elevation) {
        std::ostringstream os;
        os << "the pour joint's axis is " << rad(elevation) << " off horizontal (limit "
           << rad(config.max_axis_elevation) << "): turning it would swing the lip round "
              "rather than tip it. This is the axis a top-down grasp gives; pouring needs the "
              "side grasp";
        why = os.str();
        return false;
    }
    y_ = (axis - axis.dot(up) * up).normalized();
    x_ = y_.cross(up);
    R_v0_.col(0) = x_;
    R_v0_.col(1) = y_;
    R_v0_.col(2) = up;

    // ---- the vessel, from its marker -------------------------------------
    const double tag_distance = (tag_position - ee_start.translation()).norm();
    if (tag_distance > config.max_tag_distance) {
        std::ostringstream os;
        os << "the held vessel's marker is " << tag_distance << " m from the wrist (limit "
           << config.max_tag_distance << " m): it is not on anything the gripper holds";
        why = os.str();
        return false;
    }
    Eigen::Isometry3d T_v0 = Eigen::Isometry3d::Identity();
    T_v0.linear() = R_v0_;
    T_v0.translation() = tag_position - R_v0_ * vessel.tag_in_vessel;
    T_v_ee_ = T_v0.inverse() * ee_start;
    lip_in_vessel_ = Eigen::Vector3d(vessel.radius, 0.0, vessel.height);
    lip0_ = T_v0 * lip_in_vessel_;

    const Eigen::Vector3d closing = (ee_start.linear() * config.closing_axis_in_ee).normalized();
    if (std::abs(closing.dot(up)) > 0.5) {
        why = "the jaws close along the vessel's axis rather than across it: this is not a "
              "grasp a vessel can be poured from";
        return false;
    }
    // The jaws' centre line runs through the EE origin, and they centre what
    // they close on. So the vessel's axis lies in that plane, and how far the
    // measurement puts it outside is how wrong the measurement is.
    centering_error_ = (T_v0.translation() - ee_start.translation()).dot(closing);
    if (std::abs(centering_error_) > config.max_centering_error) {
        std::ostringstream os;
        os << "the vessel measured " << mm(centering_error_) << " off the jaws' centre line "
           << "along the direction they close (limit " << mm(config.max_centering_error)
           << "). The jaws centre what they grip, so that is measurement error, not grasp: a "
              "marker seen by one camera is off along that camera's line of sight. Check both "
              "side cameras see it, and that tag_in_vessel matches where it is stuck";
        why = os.str();
        return false;
    }

    // ---- the receiver, in pour coordinates ------------------------------
    const double R = receiver.radius;
    const Eigen::Vector3d near_rim = receiver.rim_center - x_ * R;
    const double inset0 = (lip0_ - near_rim).dot(x_);
    const double lateral0 = (lip0_ - receiver.rim_center).dot(y_);
    const double height0 = lip0_.z() - receiver.rim_center.z();
    if ((receiver.rim_center - lip0_).dot(x_) <= 0.0) {
        why = "the receiver is behind the lip: tilting this way tips the vessel away from it. "
              "Check pour_direction, or bring the vessel to the other side of the receiver";
        return false;
    }

    // ---- what must stay out of it --------------------------------------
    points_.clear();
    const double r = vessel.radius;
    const double H = vessel.height;
    const int n_wall = std::max(1, static_cast<int>(std::ceil(H / config.outline_step)));
    for (int i = 0; i <= n_wall; ++i) {
        const double z = H * i / n_wall;
        points_.emplace_back(0.0, 0.0, z - H);        // the lip-side wall
        points_.emplace_back(-2.0 * r, 0.0, z - H);   // the far wall
    }
    const int n_base = std::max(1, static_cast<int>(std::ceil(2.0 * r / config.outline_step)));
    for (int i = 0; i <= n_base; ++i) {
        points_.emplace_back(-2.0 * r * i / n_base, 0.0, -H);
    }
    std::vector<Eigen::Vector3d> hand;
    for (const auto & box : config.hand_boxes) {
        sample_box_surface(box, config.hand_step, hand);
    }
    for (const auto & p : hand) {
        points_.push_back(R_v0_.transpose() * (ee_start * p - lip0_));
    }

    // Where it all is now. A point already inside the receiver's footprint and
    // below its rim is a collision this controller did not cause and cannot
    // safely undo.
    for (const auto & d : points_) {
        const double h = height0 + d.z();
        const double lateral = std::abs(lateral0 + d.y());
        if (h >= 0.0 || lateral >= R) {
            continue;
        }
        const double edge = R - std::sqrt(R * R - lateral * lateral);
        if (inset0 + d.x() > edge) {
            std::ostringstream os;
            os << "the held vessel or the jaws already reach " << mm(inset0 + d.x() - edge)
               << " inside the receiver's rim, " << mm(-h) << " below it. Nothing is moved "
                  "from there: bring the vessel back beside the receiver first";
            why = os.str();
            return false;
        }
    }

    // ---- the height it is held at ----------------------------------------
    height_ = std::clamp(height0, config.clearance, config.max_height);
    build_at_height(inset0, lateral0, height0, max_tilt);
    const double by = std::min(config.landing_by_tilt, max_tilt);
    const double lifted_from = height_;
    if (landing_tilt_ > by + 1e-9) {
        const double needed = required_height(by);
        if (needed > config.max_height) {
            std::ostringstream os;
            os << "held " << mm(height_) << " above the rim, the stream would land in the "
               << "receiver only from " << rad(landing_tilt_) << ", and a vessel full enough "
                  "to start pouring at " << rad(by) << " would pour onto the rim first. "
               << "Catching it from there needs the lip " << mm(needed) << " up, above "
                  "max_height (" << mm(config.max_height) << "): grasp the vessel lower so the "
                  "jaws hang further below the lip, or lower landing_by_tilt for a vessel "
                  "that is never that full";
            why = os.str();
            return false;
        }
        height_ = std::max(height_, needed);
        build_at_height(inset0, lateral0, height0, max_tilt);
    }

    if (align_length_ > config.max_align_distance) {
        std::ostringstream os;
        os << "the lip is " << mm(align_length_) << " of travel from where the pour starts "
           << "(limit " << mm(config.max_align_distance) << "): it was measured "
           << mm(inset0) << " past the receiver's near rim, " << mm(lateral0)
           << " off its centre line and " << mm(height0) << " above it, and starts "
           << mm(inset_limit(0.0)) << " past it at " << mm(height_) << ". Bring the vessel "
              "closer to that; this only corrects what the pre-pour pose got slightly wrong";
        why = os.str();
        return false;
    }
    if (!std::isfinite(landing_tilt_)) {
        std::ostringstream os;
        os << "held " << mm(height_) << " above the rim, the lip never gets "
           << mm(config.landing_margin) << " inside it before the tilt bound ("
           << rad(max_tilt) << "): the vessel's own wall or a jaw below the lip keeps it out";
        why = os.str();
        return false;
    }
    if (last_landing_tilt_ - landing_tilt_ < config.scan_step) {
        std::ostringstream os;
        os << "the lip is over the receiver's mouth only at " << rad(landing_tilt_)
           << ", not over a range of tilts a pour could use: a jaw comes down onto the rim "
              "just after it gets there. Grasp the vessel lower";
        why = os.str();
        return false;
    }

    const double inset_start = inset_limit(0.0);
    // Under the side grasp the approach axis runs horizontally through the EE
    // origin and the jaws' centre, so the origin's height IS the grasp height.
    std::ostringstream os;
    os << "vessel held with the wrist axis " << mm(T_v_ee_.translation().z())
       << " above its bottom (" << mm(centering_error_) << " off the jaws' centre line); lip "
       << "measured " << mm(inset0) << " past the near rim, " << mm(lateral0) << " off centre, "
       << mm(height0) << " above; aligning " << mm(align_length_) << " to start "
       << mm(inset_start) << " past it at " << mm(height_);
    if (height_ > lifted_from + 1e-9) {
        os << " (lifted from " << mm(lifted_from) << " so the stream lands inside from "
           << rad(by) << ")";
    }
    os << "; pours into the receiver from " << rad(landing_tilt_) << " to "
       << rad(last_landing_tilt_);
    summary_ = os.str();
    planned_ = true;
    return true;
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
