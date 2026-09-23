#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace cho_controller {
namespace fr5 {
namespace pour {

//: The vessel in the gripper. Its frame, used by everything below: origin at
//: the centre of its bottom, z up its axis, x toward the lip it pours over.
struct HeldVessel {
    double radius{0.025};
    //: Bottom to rim [m].
    double height{0.070};
    //: Where the marker the cameras locate is stuck on it: its centre's
    //: horizontal distance from the vessel's axis, and its height above the
    //: bottom [m]. Only its POSITION is used -- a tag's orientation is the part
    //: of its pose that flips between frames -- and deliberately NOT which side
    //: of the vessel it is on. That depends on how the vessel stood when it was
    //: picked up and on how the grasp approached it, both of which change from
    //: one recording to the next; the jaws' centring supplies the rest.
    double tag_radius{0.0};
    double tag_height{0.0};
};

//: The vessel being poured into. Fixed, not perceived: it stands on the scale.
struct Receiver {
    //: Centre of the rim circle, base frame [m].
    Eigen::Vector3d rim_center{Eigen::Vector3d::Zero()};
    double radius{0.025};
};

//: Something else on the arm that must not hit the receiver -- the jaws. An
//: axis-aligned box in the EE frame.
struct HandBox {
    Eigen::Vector3d min{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

struct LipPathConfig {
    //: Lowest the lip is held above the receiver's rim [m].
    double clearance{0.020};
    //: Highest. The lip is held at one height for the whole pour.
    double max_height{0.080};
    //: That height [m]: `lip_height` when positive, otherwise the height the
    //: vessel was brought in at -- clamped to [clearance, max_height] either
    //: way, then lifted for landing_by_tilt. A taught pre-pour pose already
    //: says how high; a recorded one hangs the vessel 20 cm up, and pouring
    //: from max_height because of it would be a choice nobody made.
    double lip_height{0.0};
    //: How far inside the receiver's near rim the lip is brought once nothing
    //: prevents it [m].
    double inset{0.012};
    //: Kept between anything on the arm and the receiver [m].
    double gap{0.005};
    //: Lip at least this far inside the rim counts as the stream landing in
    //: the receiver [m].
    double landing_margin{0.005};
    //: The stream has to land in the receiver from this tilt on [rad]. Held
    //: too low, the jaws and the wall keep the lip out of the mouth until late
    //: in the tilt, and a fuller vessel starts pouring before then -- onto the
    //: rim. The lip is lifted, as far as max_height, until it would not.
    //: 0.5 rad covers a 100 mL beaker up to about 110 g of water.
    double landing_by_tilt{0.5};
    //: Largest correction made before tilting [m]. The pre-pour pose is still
    //: the task's; this only absorbs how far off it turned out to be.
    double max_align_distance{0.060};
    //: How far the pour axis may be off horizontal [rad]. Tilting about a
    //: steep axis swings the lip sideways instead of tipping it.
    double max_axis_elevation{0.35};
    //: How much further the marker may measure from the jaws' centre line
    //: than tag_radius puts it from the vessel's axis [m]. The jaws centre what
    //: they grip, so the axis lies on that line and the marker cannot be
    //: further from it than from the axis: any excess is measurement error.
    double max_centering_error{0.008};
    //: Distance along the approach from the EE origin to where the jaws hold
    //: [m], and how far from it the vessel's axis may come out. The marker and
    //: the centring leave two places along the jaws the axis could be, a
    //: marker's width apart; this picks the one where the jaws actually are.
    //: So it has to be right to well within tag_radius: off by more, it picks
    //: the other place, 2 * tag_radius away, and nothing can tell.
    double grasp_depth{0.29};
    double max_depth_error{0.04};
    Eigen::Vector3d approach_axis_in_ee{Eigen::Vector3d::UnitZ()};
    //: A marker further than this from the EE is not on anything it holds [m].
    double max_tag_distance{0.5};
    Eigen::Vector3d closing_axis_in_ee{Eigen::Vector3d::UnitX()};
    std::vector<HandBox> hand_boxes;
    //: Sampling of the vessel's outline and of the hand boxes' surfaces [m].
    //: Both have to stay under `gap`: the constraint is enforced at samples.
    double outline_step{0.001};
    double hand_step{0.004};
    //: Tilt resolution of the landing-range scan done once at plan [rad].
    double scan_step{0.02};

    bool validate(std::string & why) const;
};

/**
 * Where a held vessel's lip goes as it tilts, worked out from where the vessel
 * actually sits in the gripper.
 *
 * Tilting the pour joint alone swings the lip on an arc about the grasp: from
 * a 30-50 mm grasp on a 100 mL beaker it drops 30-39 mm and moves 5-22 mm out
 * over the working range, so where the stream lands depends on where the jaws
 * happened to close. Given where the vessel is -- measured, relative to the
 * EE -- the pour can instead hold the lip at a chosen height above the
 * receiver's rim and bring it in over the mouth, rotating the vessel about the
 * lip rather than about the wrist.
 *
 * WHY THE LIP CANNOT SIMPLY START OVER THE MOUTH. Upright, a vessel hangs
 * straight down from its lip; with the lip inside the receiver's rim, its body
 * would be sitting on it. The lip may only come in as far as the tilt swings
 * the body back out, which is `height * tan(tilt)` for the vessel's own wall.
 * The jaws are worse: under the side grasp the pour joint tips the vessel
 * toward a jaw, so that jaw hangs just below the lip and protrudes past it.
 * The path is therefore a CONSTRAINT, re-evaluated at every tilt: the lip goes
 * as far in as `inset`, but never further than keeps every sampled point of the
 * vessel and the configured jaws out of the receiver, inflated by `gap`. Points
 * above the rim (plus `gap`) are not constrained.
 *
 * That is why the useful tilt range is an output here. `plan()` scans for the
 * tilts at which the lip is over the mouth. Both ends matter:
 *
 *   - the LOWER end is where the stream starts landing in the receiver. A
 *     vessel full enough to start pouring before it pours onto the rim, so the
 *     lip is lifted (up to max_height) until the lower end is at most
 *     `landing_by_tilt`: lifting clears the jaw below the lip over the rim,
 *     which lets the lip in sooner.
 *   - the UPPER end is where a jaw that started clear above the rim comes down
 *     onto it as the vessel tips toward it. Past there the lip would have to
 *     back out of the mouth, taking the stream with it, so the caller hands the
 *     law this end as its tilt bound.
 *
 * Coordinates. Everything about the receiver is measured along the POUR
 * DIRECTION `x` (horizontal, from the lip toward the receiver), the tilt axis
 * `y` (horizontal) and base z. "Inset" is the lip's distance past the
 * receiver's near rim along `x`, positive inside; "height" is above the rim.
 *
 * Assumptions, both checked where they can be: the vessel hangs upright at the
 * attitude it was carried in (a lean shows up as the onset arriving early or
 * late, which the law finds anyway), and the jaws' centre line runs through its
 * axis.
 *
 * No ROS, no Pinocchio: poses in, poses out. The caller turns them into joints.
 */
class LipPath
{
public:
    //: Plan from the attitude the vessel was carried in. On failure nothing is
    //: planned, `why` says what was wrong and what would fix it, and nothing
    //: should move.
    //:   ee_start      the EE pose the vessel was measured at (base frame)
    //:   pour_axis     rotation axis whose positive sense brings the lip down
    //:   tag_position  the marker's measured centre (base frame)
    //:   max_tilt      the goal's tilt bound, which the landing range caps
    bool plan(const Eigen::Isometry3d & ee_start, const Eigen::Vector3d & pour_axis,
              const Eigen::Vector3d & tag_position, double max_tilt,
              const HeldVessel & vessel, const Receiver & receiver,
              const LipPathConfig & config, std::string & why);

    [[nodiscard]] bool planned() const { return planned_; }

    //: Length of the straight-line moves that take the lip from where it was
    //: measured to where the tilt starts [m]. Zero when it is already there.
    [[nodiscard]] double align_length() const { return align_length_; }
    //: EE pose `s` metres along the alignment, orientation unchanged.
    [[nodiscard]] Eigen::Isometry3d ee_pose_aligning(double s) const;

    //: The furthest in the lip may be at `tilt` [m]; `inset` when nothing is
    //: in the way. The path the controller follows.
    [[nodiscard]] double inset_limit(double tilt) const;
    //: EE pose with the vessel tilted by `tilt` and its lip `inset` past the
    //: near rim, at the planned height and centred on the receiver.
    [[nodiscard]] Eigen::Isometry3d ee_pose(double tilt, double inset) const;
    [[nodiscard]] Eigen::Isometry3d vessel_pose(double tilt, double inset) const;

    //: Tilts between which the lip is at least `landing_margin` inside the
    //: rim, i.e. between which what pours lands in the receiver [rad].
    [[nodiscard]] double landing_tilt() const { return landing_tilt_; }
    [[nodiscard]] double last_landing_tilt() const { return last_landing_tilt_; }
    //: Height above the rim the lip is held at [m]: lip_height or where it
    //: was brought in, clamped to [clearance, max_height], and lifted for
    //: landing_by_tilt.
    [[nodiscard]] double height() const { return height_; }
    [[nodiscard]] const Eigen::Vector3d & pour_direction() const { return x_; }
    [[nodiscard]] const Eigen::Vector3d & tilt_axis() const { return y_; }
    [[nodiscard]] const Eigen::Vector3d & lip_start() const { return lip0_; }
    //: The measured grasp: the EE origin in the vessel frame [m].
    [[nodiscard]] Eigen::Vector3d ee_in_vessel() const { return T_v_ee_.translation(); }
    //: The lip in the EE frame: the point the arm has to keep on the path.
    [[nodiscard]] Eigen::Vector3d lip_in_ee() const { return T_v_ee_.inverse() * lip_in_vessel_; }
    //: How far the marker measured from the jaws' centre line, and how far
    //: along it the axis came out from grasp_depth [m].
    [[nodiscard]] double marker_offset() const { return marker_offset_; }
    [[nodiscard]] double depth_error() const { return depth_error_; }
    //: One line for the log: where the lip was, where it goes, what range pours.
    [[nodiscard]] const std::string & summary() const { return summary_; }

private:
    [[nodiscard]] double inset_limit_at(double tilt, double height) const;
    //: Base-frame point at `inset` past the near rim, `lateral` along the tilt
    //: axis from the receiver's centre line, `height` above the rim.
    [[nodiscard]] Eigen::Vector3d to_base(double inset, double lateral, double height) const;
    [[nodiscard]] double required_height(double tilt) const;
    //: Alignment and landing range for the current height_.
    void build_at_height(double inset0, double lateral0, double height0, double max_tilt);

    bool planned_{false};
    LipPathConfig config_;
    HeldVessel vessel_;
    Receiver receiver_;
    Eigen::Isometry3d ee_start_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d T_v_ee_{Eigen::Isometry3d::Identity()};
    Eigen::Matrix3d R_v0_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d x_{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d y_{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d lip0_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d lip_in_vessel_{Eigen::Vector3d::Zero()};
    //: Every sampled point of the vessel and the hand, as an offset from the
    //: lip in the vessel frame at the carried attitude.
    std::vector<Eigen::Vector3d> points_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<double> waypoint_s_;
    double align_length_{0.0};
    double height_{0.0};
    double landing_tilt_{0.0};
    double last_landing_tilt_{0.0};
    double marker_offset_{0.0};
    double depth_error_{0.0};
    std::string summary_;
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
