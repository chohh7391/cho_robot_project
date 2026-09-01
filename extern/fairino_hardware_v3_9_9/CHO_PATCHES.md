# cho patches to fairino_hardware_v3_9_9

Vendored from FAIR-INNOVATION/frcobot_ros2 (`fairino_hardware_v3_9_9`) plus
`fairino_msgs`. Only `src/fairino_hardware_interface.cpp` is patched, all changes
marked `// cho patch`:

1. **robot_ip parameter** — `on_init` reads `robot_ip` from the ros2_control
   `<hardware>` parameters instead of the hardcoded `CONTROLLER_IP_ADDRESS`
   `#define` (falls back to the define when absent). Lets `fr5.config.yaml` set it.

2. **A2-velocity** — the cho controllers require a `position + velocity` state
   interface per joint, but upstream exports position only:
   - `on_init` now expects two state interfaces and checks `[1] == velocity`.
   - `export_state_interfaces` exports the velocity state.
   - `read()` fills it from `GetActualJointSpeedsDegree(1, ...)` (deg/s -> rad/s).

Command interface is unchanged (position only, `ServoJ`). Effort/torque remain
unwired (see FR5_TODO.md §3.3).
