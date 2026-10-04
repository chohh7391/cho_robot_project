// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

namespace cho_controller {
namespace fr5 {
namespace pour {

struct FlowModelConfig {
    //: Outflow per radian past the onset [g/s/rad], before the pour has shown
    //: its own, and how unsure that guess is.
    double prior_gain{40.0};
    double prior_gain_sd{40.0};
    //: How unsure the onset the seek found is [rad]. The seek stops a delay
    //: after the flow started, and a lip that holds the water back moves the
    //: flow's start past the onset proper.
    double prior_onset_sd{0.05};
    //: How far the onset moves per gram poured [rad/g], and how unsure. A
    //: straight-walled vessel has to tip further the emptier it gets -- 0.0053
    //: rad/g measured on the FR5 cell's 100 mL beaker -- and it is what makes a
    //: flow held at one tilt die away by itself.
    double prior_rise{0.0};
    double prior_rise_sd{0.01};
    //: Spread of one fitted flow sample about the model [g/s].
    double flow_noise{0.5};
    //: Pairs the model has to have fitted before its forecast may stop the
    //: bulk; until then, flow x delay stands in. 0 is off, the default: on the
    //: rig (2026-09-28) the priors said ~8 g was coming two samples after the
    //: onset when 2.3 g was, but in the sim waiting for 3 pairs made 10 g
    //: pours 2.6 g over at worst where they had been 0.46. The fix that held in
    //: both was a lower prior_gain.
    int min_pairs{0};
};

/**
 * How fast the vessel pours at a given tilt, learned from the pour itself.
 *
 *   outflow = gain * max(0, tilt - onset(left)),   onset(left) = onset0 + rise * left
 *
 * `left` is what has left the lip since the pour began. The scale cannot say
 * what is in the air -- only what has landed -- but it can say, a delay late,
 * what the vessel did at every tilt it has been at. Each flow it reports is
 * paired with the tilt of one transport delay before it, and the three
 * coefficients of `gain*tilt - gain*onset0 - gain*rise*left` are a linear
 * regression on those pairs, started from priors and never forgetting: a pour
 * is seconds long, and every pair it produces is evidence.
 *
 * No ROS, no clock, no history of its own. The caller owns the tilt history
 * and does the pairing; this only fits and evaluates.
 */
class FlowModel
{
public:
    //: Start a pour whose flow was first seen at `onset`.
    void begin(const FlowModelConfig & config, double onset);

    //: One pair: the flow the scale reported, the tilt that produced it, and
    //: what had left the lip by then.
    void observe(double tilt, double left, double flow);

    //: Predicted outflow [g/s]. Never negative.
    [[nodiscard]] double outflow(double tilt, double left) const;

    [[nodiscard]] bool started() const { return started_; }
    [[nodiscard]] int pairs() const { return pairs_; }
    [[nodiscard]] double gain() const { return theta_[0]; }
    //: Where the model puts the onset with `left` gone [rad].
    [[nodiscard]] double onset(double left) const;

private:
    void solve();

    FlowModelConfig config_;
    bool started_{false};
    int pairs_{0};
    //: Information form: posterior precision and precision-weighted mean.
    double info_[3][3]{};
    double info_mean_[3]{};
    //: [gain, -gain*onset0, -gain*rise]
    double theta_[3]{};
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
