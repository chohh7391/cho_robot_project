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

#include "cho_controller_fr5/pour/flow_model.hpp"

#include <algorithm>
#include <cmath>

namespace cho_controller {
namespace fr5 {
namespace pour {

namespace {

// Inverse of a symmetric positive-definite 3x3, by cofactors. False when it is
// not invertible, in which case `out` is untouched.
bool invert3(const double m[3][3], double out[3][3])
{
    const double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (!std::isfinite(det) || std::abs(det) < 1e-300) {
        return false;
    }
    const double inv = 1.0 / det;
    out[0][0] = c00 * inv;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
    out[1][0] = c01 * inv;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
    out[2][0] = c02 * inv;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
    return true;
}

} // namespace

void FlowModel::begin(const FlowModelConfig & config, double onset)
{
    config_ = config;
    pairs_ = 0;
    const double g = config.prior_gain;
    const double s = config.prior_rise;
    theta_[0] = g;
    theta_[1] = -g * onset;
    theta_[2] = -g * s;

    // The priors are on gain, onset and rise, which is what anyone can guess;
    // the regression is on theta = f(gain, onset, rise). Carried through the
    // Jacobian of f at the prior, which is exact for the gain alone and a
    // first-order spread for the products.
    const double J[3][3] = {{1.0, 0.0, 0.0}, {-onset, -g, 0.0}, {-s, 0.0, -g}};
    const double var[3] = {config.prior_gain_sd * config.prior_gain_sd,
                           config.prior_onset_sd * config.prior_onset_sd,
                           config.prior_rise_sd * config.prior_rise_sd};
    double cov[3][3]{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                cov[i][j] += J[i][k] * var[k] * J[j][k];
            }
        }
    }
    // A zero spread is legitimate -- "this vessel's onset does not move" -- and
    // would make the covariance singular, so every variance gets a floor far
    // below anything a pour could resolve.
    for (int i = 0; i < 3; ++i) {
        cov[i][i] += 1e-12 * (1.0 + std::abs(cov[i][i]));
    }
    started_ = invert3(cov, info_);
    if (!started_) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        info_mean_[i] = 0.0;
        for (int j = 0; j < 3; ++j) {
            info_mean_[i] += info_[i][j] * theta_[j];
        }
    }
}

void FlowModel::observe(double tilt, double left, double flow)
{
    if (!started_ || !std::isfinite(tilt) || !std::isfinite(left) || !std::isfinite(flow)) {
        return;
    }
    const double x[3] = {tilt, 1.0, left};
    const double w = 1.0 / (config_.flow_noise * config_.flow_noise);
    for (int i = 0; i < 3; ++i) {
        info_mean_[i] += w * x[i] * flow;
        for (int j = 0; j < 3; ++j) {
            info_[i][j] += w * x[i] * x[j];
        }
    }
    ++pairs_;
    solve();
}

void FlowModel::solve()
{
    double cov[3][3];
    if (!invert3(info_, cov)) {
        return;
    }
    double theta[3]{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            theta[i] += cov[i][j] * info_mean_[j];
        }
    }
    // A vessel that pours LESS the further it is tipped is not a vessel; a fit
    // that says so has been fooled -- most often by a lip that held water back
    // and then let go -- and keeping the last physical fit is the safer error.
    if (!std::isfinite(theta[0]) || !std::isfinite(theta[1]) || !std::isfinite(theta[2]) ||
        theta[0] <= 0.0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        theta_[i] = theta[i];
    }
}

double FlowModel::outflow(double tilt, double left) const
{
    if (!started_) {
        return 0.0;
    }
    const double q = theta_[0] * tilt + theta_[1] + theta_[2] * left;
    return std::isfinite(q) ? std::max(0.0, q) : 0.0;
}

double FlowModel::onset(double left) const
{
    return theta_[0] > 0.0 ? -(theta_[1] + theta_[2] * left) / theta_[0] : 0.0;
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
