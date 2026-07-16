# Multi-PC Setup (PC1 & PC2)

- PC1: discovery server + bringup (controllers, action servers)
- PC2: clients (action_client.py, task manager, `ros2` CLI, rqt, rviz)

Communication runs over Tailscale + Fast DDS Discovery Server, managed by
[`dds/dds_mode.sh`](../dds/dds_mode.sh).

## One-time setup (both PCs)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up   # login with the same account
tailscale ip -4     # check each PC's IP
```

Set `_DDS_SERVER_IP` in `dds/dds_mode.sh` to PC1's Tailscale IP.

## Daily usage

Every terminal picks a DDS mode by sourcing `dds/dds_mode.sh`. Terminals only see
each other within the same mode family: `local`↔`local`, `server`↔`client`
(a `local` terminal never talks to the others).

1. Discovery server (PC1, keep this terminal running):

```bash
fastdds discovery --server-id 0 -l 0.0.0.0 -p 11811
```

2. Bringup (PC1):

```bash
source ~/ros2_ws/src/cho_robot_project/dds/dds_mode.sh server
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=<mode> controller_name:=<controller>
```

3. Clients / CLI (PC2, or extra PC1 terminals for introspection):

```bash
source ~/ros2_ws/src/cho_robot_project/dds/dds_mode.sh client
source ~/ros2_ws/install/setup.bash
python3 cho_task_manager/python/action_client.py   # or: ros2 topic list, rqt, rviz ...
```

Client mode loads `dds/super_client.xml` (Fast DDS `SUPER_CLIENT`), so graph
introspection — `ros2 topic list`, action-server auto-discovery — works over the
discovery server (a plain discovery-server client only learns about endpoints
matching its own readers/writers, so graph queries come back empty).

Single-PC work: `source dds/dds_mode.sh local`. Check the current mode anytime
with `dds_status`.
