// The lip path, against a synthetic side grasp of a 100 mL beaker next to the
// receiver on the scale. No robot: the EE pose is constructed, and every
// clearance is checked against the receiver's real cylinder by sampling the
// vessel and the jaws independently of how LipPath samples them.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "cho_controller_fr5/pour/lip_path.hpp"

using cho_controller::fr5::pour::HandBox;
using cho_controller::fr5::pour::HeldVessel;
using cho_controller::fr5::pour::LipPath;
using cho_controller::fr5::pour::LipPathConfig;
using cho_controller::fr5::pour::Receiver;

namespace {

constexpr double kR = 0.025;      // both beakers: 100 mL, 50 mm across
constexpr double kH = 0.070;
constexpr double kRimZ = 0.180;   // the receiver's rim, as measured on the cell
constexpr double kJawDepth = 0.245;

// The FR5's side grasp: the jaws close along EE x, the approach is EE z, and
// here EE z points along base +y, so the pour joint's axis is base +y and a
// positive tilt brings the +x side of the vessel down -- toward the receiver.
struct Scene {
    Eigen::Vector3d vessel_origin{0.45, 0.32, 0.15};   // bottom centre, base frame
    double grasp_height{0.030};                        // jaws' centre above the bottom
    // Where the marker is stuck, as an offset from the bottom centre in BASE
    // axes: on a card beyond the fingertips (the approach is base +y), which is
    // the one side of a side-gripped beaker the jaws leave free and a camera in
    // front can see.
    Eigen::Vector3d tag_offset{0.0, kR + 0.003, 0.045};
    Eigen::Vector3d tag_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d pour_axis{Eigen::Vector3d::UnitY()};
    Receiver receiver;
    HeldVessel vessel;
    LipPathConfig config;
    double max_tilt{1.2};

    Scene()
    {
        receiver.rim_center = Eigen::Vector3d(0.52, 0.32, kRimZ);
        receiver.radius = kR;
        vessel.radius = kR;
        vessel.height = kH;
        config.grasp_depth = kJawDepth;
    }

    Eigen::Isometry3d ee() const
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear().col(0) = Eigen::Vector3d::UnitX();
        T.linear().col(1) = -Eigen::Vector3d::UnitZ();
        T.linear().col(2) = Eigen::Vector3d::UnitY();
        const Eigen::Vector3d jaw_centre = vessel_origin + Eigen::Vector3d(0, 0, grasp_height);
        T.translation() = jaw_centre - kJawDepth * T.linear().col(2);
        return T;
    }

    Eigen::Vector3d tag() const { return vessel_origin + tag_offset + tag_error; }

    //: Two jaws closed on the vessel: `thickness` past its wall, `half_height`
    //: above and below the grasp, the AG-95's 110 mm along the approach.
    void with_jaws(double thickness = 0.012, double half_height = 0.020)
    {
        HandBox lip_side;
        lip_side.min = Eigen::Vector3d(kR, -half_height, 0.1904);
        lip_side.max = Eigen::Vector3d(kR + thickness, half_height, 0.3008);
        HandBox far_side;
        far_side.min = Eigen::Vector3d(-kR - thickness, -half_height, 0.1904);
        far_side.max = Eigen::Vector3d(-kR, half_height, 0.3008);
        config.hand_boxes = {lip_side, far_side};
    }

    bool plan(LipPath & path, std::string & why) const
    {
        vessel_with_tag_ = vessel;
        vessel_with_tag_.tag_radius = tag_offset.head<2>().norm();
        vessel_with_tag_.tag_height = tag_offset.z();
        return path.plan(ee(), pour_axis, tag(), max_tilt, vessel_with_tag_, receiver, config, why);
    }

    mutable HeldVessel vessel_with_tag_;
};

// Deepest incursion into the receiver of anything at this pose: the vessel's
// whole surface and, if given, the jaws. Positive means a collision.
double incursion(const Eigen::Isometry3d & vessel_pose, const Eigen::Isometry3d & ee_pose,
                 const std::vector<HandBox> & jaws, const Receiver & receiver)
{
    std::vector<Eigen::Vector3d> points;
    for (int k = 0; k <= 70; ++k) {
        const double z = kH * k / 70.0;
        for (int a = 0; a < 72; ++a) {
            const double phi = 2.0 * M_PI * a / 72.0;
            points.push_back(vessel_pose * Eigen::Vector3d(kR * std::cos(phi), kR * std::sin(phi), z));
        }
    }
    for (int i = 0; i <= 25; ++i) {
        for (int a = 0; a < 72; ++a) {
            const double rr = kR * i / 25.0;
            const double phi = 2.0 * M_PI * a / 72.0;
            points.push_back(vessel_pose * Eigen::Vector3d(rr * std::cos(phi), rr * std::sin(phi), 0.0));
        }
    }
    for (const auto & box : jaws) {
        const Eigen::Vector3d size = box.max - box.min;
        for (int i = 0; i <= 20; ++i) {
            for (int j = 0; j <= 20; ++j) {
                for (int k = 0; k <= 40; ++k) {
                    points.push_back(ee_pose * (box.min + Eigen::Vector3d(size.x() * i / 20.0,
                                                                          size.y() * j / 20.0,
                                                                          size.z() * k / 40.0)));
                }
            }
        }
    }
    double worst = -1.0;
    for (const auto & p : points) {
        const double below = receiver.rim_center.z() - p.z();
        const double inside = receiver.radius - (p - receiver.rim_center).head<2>().norm();
        if (below > 0.0 && inside > 0.0) {
            worst = std::max(worst, std::min(below, inside));
        }
    }
    return worst;
}

Eigen::Vector3d lip_of(const LipPath & path, double tilt)
{
    return path.vessel_pose(tilt, path.inset_limit(tilt)) * Eigen::Vector3d(kR, 0.0, kH);
}

}  // namespace

TEST(LipPath, RecoversTheGraspItWasGiven)
{
    Scene scene;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    // The EE origin in the vessel frame: on the jaws' centre line, kJawDepth
    // back along the approach, at the grasp height.
    const Eigen::Vector3d ee = path.ee_in_vessel();
    EXPECT_NEAR(ee.x(), 0.0, 1e-9);
    EXPECT_NEAR(ee.y(), -kJawDepth, 1e-9);
    EXPECT_NEAR(ee.z(), scene.grasp_height, 1e-9);
    EXPECT_NEAR(path.depth_error(), 0.0, 1e-9);
    EXPECT_NEAR(path.marker_offset(), 0.0, 1e-9);
    EXPECT_TRUE(path.pour_direction().isApprox(Eigen::Vector3d::UnitX()));
}

TEST(LipPath, HoldsTheLipAtOneHeightOnTheReceiversCentreLine)
{
    Scene scene;
    scene.vessel_origin.z() = kRimZ + 0.040 - kH;   // lip 40 mm above the rim
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_NEAR(path.height(), 0.040, 1e-9);
    for (double tilt = -0.3; tilt <= 1.2; tilt += 0.01) {
        const Eigen::Vector3d lip = lip_of(path, tilt);
        EXPECT_NEAR(lip.z(), kRimZ + 0.040, 1e-9) << "tilt " << tilt;
        EXPECT_NEAR(lip.y(), 0.32, 1e-9) << "tilt " << tilt;
    }
}

TEST(LipPath, FollowsTheWallLawWhenNothingElseIsHeld)
{
    // Upright, the vessel hangs from its lip; the lip can come in only as far
    // as the tilt swings the wall below it back out past the rim.
    Scene scene;
    scene.vessel_origin.z() = kRimZ + 0.030 - kH;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    const double h = path.height();
    const double gap = scene.config.gap;
    for (double tilt = 0.05; tilt <= 1.2; tilt += 0.05) {
        const double law = std::min(scene.config.inset, (h - gap) * std::tan(tilt) - gap);
        EXPECT_NEAR(path.inset_limit(tilt), law, scene.config.outline_step * std::sin(tilt) + 1e-9)
            << "tilt " << tilt;
    }
    EXPECT_NEAR(path.inset_limit(0.0), -gap, 1e-9);
}

TEST(LipPath, NeverPutsTheVesselIntoTheReceiverAtAnyTilt)
{
    for (const double height : {0.020, 0.040, 0.060}) {
        Scene scene;
        scene.vessel_origin.z() = kRimZ + height - kH;
        LipPath path;
        std::string why;
        ASSERT_TRUE(scene.plan(path, why)) << "height " << height << ": " << why;
        for (double tilt = -0.3; tilt <= 1.2; tilt += 0.01) {
            const double inset = path.inset_limit(tilt);
            EXPECT_LT(incursion(path.vessel_pose(tilt, inset), path.ee_pose(tilt, inset), {},
                                scene.receiver), 0.0)
                << "height " << height << " tilt " << tilt;
        }
    }
}

TEST(LipPath, NeverPutsTheJawsIntoTheReceiverAtAnyTilt)
{
    // The jaw on the lip side hangs just below the lip and stands 37 mm past the
    // wall. It, not the glass, is what limits how far in the lip may come.
    for (const double height : {0.040, 0.060, 0.080}) {
        Scene scene;
        scene.with_jaws();
        scene.vessel_origin.z() = kRimZ + height - kH;
        LipPath path;
        std::string why;
        ASSERT_TRUE(scene.plan(path, why)) << "height " << height << ": " << why;
        for (double tilt = -0.3; tilt <= 1.2; tilt += 0.01) {
            const double inset = path.inset_limit(tilt);
            EXPECT_LT(incursion(path.vessel_pose(tilt, inset), path.ee_pose(tilt, inset),
                                scene.config.hand_boxes, scene.receiver), 0.0)
                << "height " << height << " tilt " << tilt;
        }
    }
}

TEST(LipPath, TheLipIsOverTheMouthThroughoutTheRangeItReports)
{
    Scene scene;
    scene.with_jaws();
    scene.vessel_origin.z() = kRimZ + 0.060 - kH;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    ASSERT_LT(path.landing_tilt(), path.last_landing_tilt());
    for (double tilt = path.landing_tilt(); tilt <= path.last_landing_tilt(); tilt += 0.01) {
        const Eigen::Vector3d lip = lip_of(path, tilt);
        const double inside = kR - (lip - scene.receiver.rim_center).head<2>().norm();
        EXPECT_GE(inside, scene.config.landing_margin - 1e-9) << "tilt " << tilt;
    }
}

TEST(LipPath, CapsTheTiltWhereAJawComesDownOntoTheRim)
{
    // Held 75 mm up, everything clears the rim upright, so the lip goes straight
    // in -- with the lip-side jaw (the URDF's 37 mm envelope) standing out over
    // the mouth ahead of it. Tipping the vessel lowers that jaw, and at about
    // 0.14 rad its bottom corner reaches the rim. Past there the lip would have
    // to back 40 mm out of the mouth to keep it clear, taking the stream with
    // it, so the range the law is given ends there.
    Scene scene;
    scene.with_jaws(0.0366, 0.0254);
    scene.vessel_origin.z() = kRimZ + 0.075 - kH;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_NEAR(path.inset_limit(0.0), scene.config.inset, 1e-9);
    EXPECT_LT(path.last_landing_tilt(), 0.2);
    EXPECT_LT(path.inset_limit(path.last_landing_tilt() + 0.05), -0.02);
}

TEST(LipPath, AlignsWithoutTurningAndEndsWhereTheTiltStarts)
{
    Scene scene;
    scene.with_jaws();
    // Brought in 8 mm off the centre line and a little low.
    scene.vessel_origin += Eigen::Vector3d(-0.030, 0.008, kRimZ + 0.050 - kH - 0.15);
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    ASSERT_GT(path.align_length(), 0.0);
    EXPECT_TRUE(path.ee_pose_aligning(0.0).isApprox(scene.ee(), 1e-12));
    const Eigen::Isometry3d end = path.ee_pose_aligning(path.align_length());
    EXPECT_TRUE(end.isApprox(path.ee_pose(0.0, path.inset_limit(0.0)), 1e-9));
    for (double s = 0.0; s <= path.align_length(); s += 0.001) {
        const Eigen::Isometry3d ee = path.ee_pose_aligning(s);
        EXPECT_TRUE(ee.linear().isApprox(scene.ee().linear(), 1e-12));
        // The scene's vessel frame is the base frame's orientation, so the
        // vessel moves exactly as the EE does.
        Eigen::Isometry3d vessel = Eigen::Isometry3d::Identity();
        vessel.translation() = scene.vessel_origin + (ee.translation() - scene.ee().translation());
        EXPECT_LT(incursion(vessel, ee, scene.config.hand_boxes, scene.receiver), 0.0) << "s " << s;
    }
}

TEST(LipPath, BacksOutBeforeDescendingPastTheRim)
{
    // Brought in 90 mm up with the lip 5 mm short of the rim: the lip-side jaw
    // stands 7 mm over the mouth, but 30 mm above the rim, so nothing touches.
    // The pour is held at 50 mm, where that jaw hangs 10 mm below the rim.
    // Coming straight down would put it into the receiver; the alignment has to
    // back out first.
    Scene scene;
    scene.with_jaws();
    scene.vessel_origin = Eigen::Vector3d(0.495 - 0.005 - kR, 0.32, kRimZ + 0.090 - kH);
    scene.config.max_height = 0.050;
    scene.config.landing_by_tilt = 1.1;
    scene.config.max_align_distance = 0.10;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_NEAR(path.height(), 0.050, 1e-9);
    for (double s = 0.0; s <= path.align_length(); s += 0.0005) {
        const Eigen::Isometry3d ee = path.ee_pose_aligning(s);
        Eigen::Isometry3d vessel = Eigen::Isometry3d::Identity();
        vessel.translation() = scene.vessel_origin + (ee.translation() - scene.ee().translation());
        EXPECT_LT(incursion(vessel, ee, scene.config.hand_boxes, scene.receiver), 0.0) << "s " << s;
    }
}

TEST(LipPath, FindsTheGraspWhicheverSideOfTheBeakerTheMarkerIsOn)
{
    // Which side of the beaker faces where depends on how it stood when it was
    // picked up and on the recording's approach. None of it is configured: the
    // same beaker, grasped the same way, comes out the same.
    for (const double azimuth : {90.0, -90.0, 45.0, 135.0, -60.0, 20.0}) {
        Scene scene;
        const double a = azimuth * M_PI / 180.0;
        scene.tag_offset = Eigen::Vector3d((kR + 0.003) * std::cos(a), (kR + 0.003) * std::sin(a),
                                           0.045);
        LipPath path;
        std::string why;
        ASSERT_TRUE(scene.plan(path, why)) << "azimuth " << azimuth << ": " << why;
        EXPECT_NEAR(path.ee_in_vessel().x(), 0.0, 1e-9) << "azimuth " << azimuth;
        EXPECT_NEAR(path.ee_in_vessel().y(), -kJawDepth, 1e-9) << "azimuth " << azimuth;
        EXPECT_NEAR(path.ee_in_vessel().z(), scene.grasp_height, 1e-9) << "azimuth " << azimuth;
    }
}

TEST(LipPath, TakesAPoseAlreadyOnTheVesselsAxisAtFaceValue)
{
    // tag_radius 0: the object table applied the marker's known offset in its
    // own yaw, so the pose is the bottom centre. The depth is then read off it
    // directly -- an error along the jaws comes through one for one, where a
    // bare radius with the marker off to one side would amplify it.
    Scene scene;
    scene.tag_offset = Eigen::Vector3d::Zero();
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_NEAR(path.ee_in_vessel().x(), 0.0, 1e-9);
    EXPECT_NEAR(path.ee_in_vessel().y(), -kJawDepth, 1e-9);
    EXPECT_NEAR(path.ee_in_vessel().z(), scene.grasp_height, 1e-9);

    scene.tag_error = Eigen::Vector3d(0.0, 0.004, 0.0);   // 4 mm further along the jaws
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_NEAR(path.ee_in_vessel().y(), -kJawDepth - 0.004, 1e-9);
    EXPECT_NEAR(path.depth_error(), 0.004, 1e-9);
}

TEST(LipPath, RefusesAVesselThatIsNotWhereTheJawsHold)
{
    // A marker 60 mm further along than any beaker between these jaws could put
    // it: something else's, or a beaker standing beyond the fingertips.
    Scene scene;
    scene.tag_error = Eigen::Vector3d(0.0, 0.060, 0.0);
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("along the jaws from where"), std::string::npos) << why;
}

TEST(LipPath, RefusesAMeasurementThatPutsTheVesselOffTheJawsCentreLine)
{
    // The jaws centre what they close on, so a marker measured further from
    // their centre line than it is stuck from the axis is a measurement wrong
    // by the difference.
    Scene scene;
    scene.tag_error = Eigen::Vector3d(0.040, 0.0, 0.0);
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("centre line"), std::string::npos) << why;
    EXPECT_FALSE(path.planned());
}

TEST(LipPath, ComesAroundFromAVesselCarriedInAboveTheReceiver)
{
    // Where a recorded pour starts: the vessel hangs over the receiver, 150 mm
    // up, lip well past its centre. Nothing is below the rim, so the lip backs
    // out above it, comes down on the near side and starts from there.
    Scene scene;
    scene.with_jaws();
    scene.vessel_origin = Eigen::Vector3d(0.52, 0.32, kRimZ + 0.150);
    scene.config.max_align_distance = 0.30;
    scene.config.lip_height = 0.050;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_GT(path.align_length(), 0.15);
    // Brought down to lip_height, not to max_height where the start's 220 mm
    // would have been clamped -- and lifted only as far as landing needs.
    EXPECT_GE(path.height(), 0.050);
    EXPECT_LT(path.height(), 0.080);
    EXPECT_LT(path.inset_limit(0.0), 0.0);
    for (double s = 0.0; s <= path.align_length(); s += 0.001) {
        const Eigen::Isometry3d ee = path.ee_pose_aligning(s);
        Eigen::Isometry3d vessel = Eigen::Isometry3d::Identity();
        vessel.translation() = scene.vessel_origin + (ee.translation() - scene.ee().translation());
        EXPECT_LT(incursion(vessel, ee, scene.config.hand_boxes, scene.receiver), 0.0) << "s " << s;
    }
    EXPECT_NE(path.summary().find("toward heading 0 deg"), std::string::npos) << path.summary();
}

TEST(LipPath, RefusesToMoveAVesselAlreadyOverTheRim)
{
    Scene scene;
    scene.vessel_origin = Eigen::Vector3d(0.49, 0.32, kRimZ - 0.03);
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("already reach"), std::string::npos) << why;
}

TEST(LipPath, RefusesAnAlignmentLongerThanItsLimit)
{
    Scene scene;
    scene.vessel_origin.x() -= 0.10;
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("of travel"), std::string::npos) << why;
}

TEST(LipPath, RefusesAPourAxisFarFromHorizontal)
{
    Scene scene;
    scene.pour_axis = Eigen::Vector3d(0.0, 0.3, 1.0);
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("off horizontal"), std::string::npos) << why;
}

TEST(LipPath, LiftsALipHeldTooLowForTheStreamToLandEarlyEnough)
{
    // 20 mm up with the jaws on, the lip first gets over the mouth at 1.0 rad.
    // A beaker that starts pouring at 0.65 rad (100 g of water) would pour onto
    // the rim for a third of a radian. Lifting clears the jaw over the rim.
    Scene scene;
    scene.with_jaws();
    scene.vessel_origin.z() = kRimZ + 0.020 - kH;
    LipPath path;
    std::string why;
    ASSERT_TRUE(scene.plan(path, why)) << why;
    EXPECT_GT(path.height(), 0.020 + 0.01);
    EXPECT_LE(path.landing_tilt(), scene.config.landing_by_tilt + scene.config.scan_step);
    EXPECT_NE(path.summary().find("lifted from 20.0 mm"), std::string::npos) << path.summary();
    for (double tilt = -0.3; tilt <= 1.2; tilt += 0.01) {
        const double inset = path.inset_limit(tilt);
        EXPECT_LT(incursion(path.vessel_pose(tilt, inset), path.ee_pose(tilt, inset),
                            scene.config.hand_boxes, scene.receiver), 0.0) << "tilt " << tilt;
    }
}

TEST(LipPath, RefusesWhenNoHeightItMayUseLetsTheStreamLandInTime)
{
    Scene scene;
    scene.with_jaws();
    scene.vessel_origin.z() = kRimZ + 0.020 - kH;
    scene.config.max_height = 0.030;
    LipPath path;
    std::string why;
    EXPECT_FALSE(scene.plan(path, why));
    EXPECT_NE(why.find("pour onto the rim first"), std::string::npos) << why;
}

TEST(LipPath, RejectsASampleSpacingWiderThanTheGapItKeeps)
{
    LipPathConfig config;
    config.hand_step = 2.0 * config.gap;
    std::string why;
    EXPECT_FALSE(config.validate(why));
    EXPECT_NE(why.find("gap"), std::string::npos) << why;
}
