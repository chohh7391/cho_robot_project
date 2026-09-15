#pragma once

#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64.hpp>

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/servers/pour_action_server.hpp"

namespace cho_controller {
namespace fr5 {

/**
 * Tilt a held vessel until a scale says enough has come out, then set it back.
 *
 * The other FR5 controllers follow a path someone else decided on. This one has
 * no path: how far the wrist turns next depends on a weight that arrives while
 * the goal runs, which is what makes the pour adaptive and what makes it a
 * controller rather than a trajectory.
 *
 * The law is the one the simulated pours use (sdl_project's tamp_server): a PD
 * on the weight error, convolution-shaped into a wrist angular velocity, rate
 * limited, integrated into the pour joint, and stopped either by the weight or
 * by an absolute tilt bound. It is deliberately sampled at `control_period`
 * (0.04 s) rather than at the controller rate, so that the law running on
 * hardware is the same law, at the same sample rate, as the one running in
 * simulation -- a pour reported from both has to be one controller, not two
 * implementations that drifted. Between law samples the commanded angle is
 * integrated at the controller rate, which is a zero-order hold on the
 * commanded velocity: the same trajectory, emitted smoothly.
 *
 * The pour joint is the one whose axis tilts the vessel, which is the last
 * joint under the AG-95's SIDE grasp (its roll axis runs through the grasp, so
 * vessel tilt tracks it 1:1). Under a top-down grasp that same rotation is yaw
 * and pours nothing, so `pour_joint` is a parameter and not an assumption.
 *
 * Safety, in the order it is checked every cycle:
 *   - a stale scale stops the tilt. A pour with no weight feedback is not an
 *     adaptive pour, and continuing to tilt on the last number it heard is the
 *     failure that empties a vessel onto the bench.
 *   - the tilt never leaves [theta_start - max_tilt, theta_start + max_tilt].
 *   - the commanded angle stays inside the joint's own limits.
 *   - no single cycle moves the command by more than max_delta_q.
 *   - a non-finite command is never written; the previous one is held.
 *
 * However the pour ends -- reached, capped, timed out, cancelled, scale lost --
 * the vessel is brought back to the attitude it was carried in before the goal
 * finishes. The steps after a pour carry the vessel to its placement, and they
 * were planned for an upright one.
 */
class PouringController : public FR5BaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    enum class Phase { Idle, Pouring, Untilt };

    struct ScaleSample {
        double grams{0.0};
        double stamp{0.0};
        bool valid{false};
    };

    bool assign_parameters();
    void build_shaping_kernel();
    // One law sample: PD on the weight error, convolution-shaped, rate limited.
    double shaped_velocity(double error);
    void write_command(const Eigen::VectorXd & q_cmd);
    void hold_reference();
    void begin_untilt(bool succeeded, const std::string & reason);
    void finish(double grams);

    std::shared_ptr<FR5PourActionServer> action_server_;

    // ---- parameters ----
    std::string scale_topic_{"/scale/grams"};
    std::string pour_joint_;
    double scale_timeout_{0.5};
    double scale_startup_grace_{2.0};
    double control_period_{0.04};
    double kp_{0.008};
    double kd_{0.002};
    double kernel_horizon_{2.5};
    double shaping_freq_{0.8};
    double shaping_decay_{1.2};
    double kernel_alpha_{0.15};
    double max_tilt_rate_{0.5};
    double max_tilt_{2.0};
    double weight_tolerance_{0.5};
    double max_delta_q_{0.005};

    // ---- scale ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scale_sub_;
    realtime_tools::RealtimeBuffer<ScaleSample> scale_buffer_;

    // ---- law state ----
    std::vector<double> kernel_;
    std::vector<double> pd_history_;   // circular, oldest overwritten
    std::size_t pd_head_{0};
    std::size_t pd_count_{0};
    double prev_error_{0.0};
    double law_accumulator_{0.0};
    double commanded_rate_{0.0};

    // ---- goal state ----
    Eigen::VectorXd q_ref_;
    int pour_index_{-1};
    Phase phase_{Phase::Idle};
    double theta_{0.0};
    double theta_start_{0.0};
    double peak_tilt_{0.0};
    double elapsed_{0.0};
    bool pending_success_{false};
    std::string pending_reason_;
    double last_grams_{0.0};
    bool scale_seen_{false};
    double feedback_accumulator_{0.0};
};

} // namespace fr5
} // namespace cho_controller
