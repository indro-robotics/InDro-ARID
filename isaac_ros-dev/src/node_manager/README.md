# node_manager

ROS2 package that manages the lifecycle of external pipelines (subprocesses) via ROS2 services. Pipelines are defined in a YAML config, started/stopped on demand, and monitored by a watchdog. Each pipeline's state is published as a latched `Bool` topic.

---

## Build & Run

```bash
colcon build --packages-select node_manager
source install/setup.bash

# Standalone
ros2 run node_manager node_manager

# Via launch file
ros2 launch px4_state_machine px4_state_machine.launch.py
```

---

## Config

`config/pipelines.yaml` — add pipelines here:

```yaml
pipelines:
  my_pipeline:
    command: "ros2 launch my_package my_launch.py"
  another_pipeline:
    command: "python3 /path/to/script.py"
```

Logs are written to `share/node_manager/logs/<pipeline_name>/`.

---

## Services

### Per-pipeline

| Service | Type | Description |
|---|---|---|
| `/node_manager/<name>` | `std_srvs/SetBool` | `true` = start, `false` = stop |
| `/node_manager/<name>/status` | `std_srvs/Trigger` | Query running state and PID |

```bash
# Start
ros2 service call /node_manager/<name> std_srvs/srv/SetBool "{data: true}"

# Stop
ros2 service call /node_manager/<name> std_srvs/srv/SetBool "{data: false}"

# Status
ros2 service call /node_manager/<name>/status std_srvs/srv/Trigger
```

### Global

| Service | Type | Description |
|---|---|---|
| `/node_manager/status_all` | `std_srvs/Trigger` | Status of all pipelines |
| `/node_manager/stop_all` | `std_srvs/Trigger` | Stop all running pipelines |

```bash
ros2 service call /node_manager/status_all std_srvs/srv/Trigger
ros2 service call /node_manager/stop_all std_srvs/srv/Trigger
```

---

## Topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/node_manager/<name>/alive` | `std_msgs/Bool` | Latched | `true` while pipeline is running |

```bash
ros2 topic echo /node_manager/<name>/alive
```

---

## Key Functions

| Function | Description |
|---|---|
| `_load_config()` | Parses `pipelines.yaml` at startup |
| `_enable_pipeline(name)` | Spawns subprocess in its own process group; opens log file |
| `_disable_pipeline(name)` | SIGTERM → 5s wait → SIGKILL; closes log file |
| `_watchdog_tick()` | Runs every 3s; detects crashed pipelines and updates alive topics |
| `shutdown_all()` | Stops all pipelines on node shutdown |