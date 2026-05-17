# lunar

Unified CLI for this workspace (simulation, operator UIs, ROS helpers).

## Quick start

```bash
cd lunar
uv sync
uv pip install -e .

# Simulation + Foxglove (from repo root)
uv run lunar sim --world ../gz_worlds/arena1.world --port 8765
```

### Physical robot + React dashboard

From the **repo root** (after `uv pip install -e .` from `lunar/`):

```bash
uv run lunar build
uv run lunar run robot
uv run lunar dashboard
```

### `lunar run` profiles

**Primary autonomy split**

| Profile | Role |
|---------|------|
| `robot` | **Manual** — frontend / drivers (teleop, sensors). Base layer; run this first on hardware. |
| `nav` | **Navigation autonomy** — perception + local terrain grid + flags + supervisor + `navigation_controller` (zone goal → dig) + **`nav_mission_executor`**. After marking the dig zone, start the mission: `ros2 topic pub --once /autonomy/nav_mission/command std_msgs/String \"data: start\"`. |
| `nav-dig` | **Nav → dig** — same stack as `nav`, plus **`dig_sequence`** in the background with `wait_for_nav_dig_arm:=true`. When the nav mission hits dig handoff it publishes `/autonomy/dig_arm` and the dig profile takes over. Use `--calibrated-rotary` (+ optional `--encoder-side`) or **`--dig-timing-ms`** for timed forward+backward legs without encoders. |
| `dig` | **Dig autonomy alone** — foreground `dig_sequence` (bench / standalone). Requires `--calibrated-rotary`, or `--dig-timing-ms` (same ms forward+back, no encoders). |
| `dig-backup` | Same as `dig`, but adds an extra end-of-cycle conveyor ON window (`end_cycle_conveyor_seconds:=5.0`) before each repeat cycle. |

**Other profiles**

| Profile | Behavior |
|---------|----------|
| `rc` | RViz + joystick (optional `--record`). |
| `autonomy` | RViz + rgb transport + main controller. |
| `test-encoder` | Drive + Arduino, 5s forward / 5s backward encoder test, then exit. |

Examples:

```bash
uv run lunar run robot
uv run lunar run nav --grid-preset standard
uv run lunar run nav-dig --dig-timing-ms 5000
uv run lunar run dig --dig-timing-ms 5000
uv run lunar run dig-backup --dig-timing-ms 5000
```

Then open `http://<robot-ip>:8501`. **`lunar dashboard`** starts the UI plus **`camera_ws`** (port **8767**, JPEG + `/sensor/ws`) and **`mission_bridge`** (port **8770**, `/mission/ws`), matching what you used to get from Streamlit without extra commands. Stop everything it spawned with **`lunar kill`**.

- UI only, no ROS sidecars: `uv run lunar dashboard --no-robot-stack`
- If **8767** or **8770** is already in use, run `lunar kill` or stop the other process before starting the dashboard again.

## Operator UIs

| Command | What it runs |
|---------|----------------|
| **`lunar dashboard`** | React **mission-control** (Vite if `pnpm` exists, else static **`dist/`**). By default also starts **`camera_ws`** (:8767) and **`mission_bridge`** (:8770). Default HTTP **8501**, background by default. |
| **`lunar mission-control`** | Same launcher as **`lunar dashboard`** (including the same default ROS sidecars). |
| **`lunar streamlit-dashboard`** | Legacy **Streamlit** Command Center + camera WebSocket helper (default port 8501 for Streamlit). |
| **`lunar mission-bridge`** | Run **only** the bridge (for split setups; usually unnecessary when using **`lunar dashboard`**). |

## ROS / robot helpers

- **`lunar check`** — Health / environment audit (replaces older `lunar doctor` references).
- **`lunar env`** / **`lunar shell-env`** — Config and exportable env.
- **`lunar topics`** — `ros2 topic list`.
- **`lunar act`** — Shortcuts for actuator / drive topics.
- **`lunar keyboard`** — Terminal teleop (Textual TUI on robot).
- **`lunar autonomy-stack`** — Shadow perception + terrain + flags + autonomy supervisor (no autonomous drive by default).
- **`lunar run`** — Background bundles **`robot`** / **`rc`** / **`autonomy`**, or foreground **`dig`** (needs **`--calibrated-rotary`**); see table above.
- **`lunar kill`** — Stop tracked background processes.

## Quality (from **repo root**)

```bash
make test-firmware   # native encoder quadrature unit tests only (g++; no ROS / uv)
make test    # lint + offline Python tests + mission-control eslint
make ci      # same + production Vite build
make tune-flags input=field_photos/in output=field_photos/out
```

Arena photos: drop images under `field_photos/in` (gitignored by default), run **`make tune-flags`**, review overlays and `field_photos/out/results.jsonl`.

## Dashboard teleop (Streamlit)

The Streamlit **Command Center** (`lunar streamlit-dashboard`) supports hold-to-drive, conveyor, bucket chain, camera pan/height, and E-stop with browser heartbeat + ROS-side watchdog.
