import streamlit as st
import streamlit.components.v1 as components
import time
import sys
import base64
import numpy as np
import pandas as pd
from pathlib import Path
import plotly.express as px
import plotly.graph_objects as go
import subprocess
import cv2

# Add both the repo root and the local package root to sys.path.
current_file = Path(__file__).resolve()
repo_root = current_file.parents[4]
package_root = repo_root / "lunar" / "src"
for path in (repo_root, package_root):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.append(path_str)

from lunar.dashboard.state import store  # noqa: E402
from lunar.dashboard.components.hardware import render_hardware_panel  # noqa: E402
from lunar.dashboard.components.teleop import render_hold_controls  # noqa: E402

BRIDGE_IMPORT_ERROR = None
UI_REFRESH_SEC = 0.2

try:
    from lunar.dashboard.bridge import start_ros_thread, get_node
except Exception as exc:
    BRIDGE_IMPORT_ERROR = exc

    def start_ros_thread():
        return None

    def get_node():
        return None


def _make_demo_camera_frame(t: float) -> np.ndarray:
    h, w = 480, 640
    x = np.linspace(0.0, 1.0, w, dtype=np.float32)
    y = np.linspace(0.0, 1.0, h, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[..., 0] = np.clip(40 + 110 * xx + 50 * np.sin(t * 0.8), 0, 255).astype(np.uint8)
    frame[..., 1] = np.clip(30 + 140 * yy + 40 * np.cos(t * 0.6), 0, 255).astype(np.uint8)
    frame[..., 2] = np.clip(90 + 80 * (1.0 - xx) + 30 * np.sin(t * 1.2), 0, 255).astype(np.uint8)

    cx = int((0.5 + 0.28 * np.sin(t * 0.35)) * (w - 1))
    cy = int((0.45 + 0.22 * np.cos(t * 0.42)) * (h - 1))
    radius = 26
    yy_i, xx_i = np.ogrid[:h, :w]
    mask = (xx_i - cx) ** 2 + (yy_i - cy) ** 2 <= radius ** 2
    frame[mask] = np.array([245, 245, 120], dtype=np.uint8)
    return frame


def _make_demo_map(t: float) -> np.ndarray:
    size = 120
    grid = np.zeros((size, size), dtype=np.int8)
    grid[12:18, 16:104] = 100
    grid[42:48, 24:96] = 65
    grid[72:78, 10:88] = 100
    grid[24:88, 90:96] = 80

    robot_x = int(22 + 20 * np.sin(t * 0.2))
    robot_y = int(58 + 16 * np.cos(t * 0.25))
    target_x = int(94 + 8 * np.sin(t * 0.15))
    target_y = int(28 + 10 * np.cos(t * 0.18))

    grid[max(robot_y - 2, 0):robot_y + 3, max(robot_x - 2, 0):robot_x + 3] = -1
    grid[max(target_y - 3, 0):target_y + 4, max(target_x - 3, 0):target_x + 4] = 35
    return grid


def _update_demo_state() -> None:
    demo_start = st.session_state.setdefault("demo_start_time", time.time())
    last_tick = st.session_state.get("demo_last_tick", 0.0)
    t = time.time() - demo_start

    odom_x = 2.4 * np.cos(t * 0.17)
    odom_y = 1.6 * np.sin(t * 0.21)
    linear_vel = 0.85 + 0.18 * np.sin(t * 0.9)
    angular_vel = 0.22 * np.cos(t * 0.7)
    baseline_vel = 0.8 + 0.12 * np.sin(t * 0.9 + 0.35)
    cpu_usage = 29.0 + 9.0 * np.sin(t * 0.33)
    ram_usage = 46.0 + 7.0 * np.cos(t * 0.19)
    cpu_temp = 54.0 + 4.5 * np.sin(t * 0.27)
    battery_voltage = 24.7 - 0.015 * t + 0.05 * np.sin(t * 0.12)
    network_latency = 18.0 + 5.0 * np.sin(t * 0.5)
    ir_left = int(420 + 130 * np.sin(t * 0.8))
    ir_right = int(470 + 120 * np.cos(t * 0.75))
    camera_height = int(55 + 18 * np.sin(t * 0.28))
    bucket_pos = int(40 + 22 * np.cos(t * 0.24))

    store.update(
        odom_x=float(odom_x),
        odom_y=float(odom_y),
        odom_z=0.0,
        linear_vel=float(linear_vel),
        angular_vel=float(angular_vel),
        baseline_vel=float(baseline_vel),
        cpu_usage=float(cpu_usage),
        ram_usage=float(ram_usage),
        cpu_temp=float(cpu_temp),
        battery_voltage=float(battery_voltage),
        network_latency=float(network_latency),
        ir_left=ir_left,
        ir_right=ir_right,
        camera_height=camera_height,
        bucket_pos=bucket_pos,
        point_cloud_density=18432,
        map_data=_make_demo_map(t),
        map_info={"resolution": 0.1, "origin": (-6.0, -6.0)},
        camera_image=_make_demo_camera_frame(t),
        camera_jpeg=None,
        recent_logs=[
            "[demo_bridge] ROS unavailable on host, rendering synthetic telemetry.",
            "[demo_mapper] Published synthetic occupancy grid.",
            "[demo_camera] Generated RGB frame 640x480.",
            "[demo_motion] Tracking virtual waypoint in map frame.",
        ],
        node_health={
            "demo_bridge": "online",
            "demo_mapper": "online",
            "demo_camera": "online",
            "demo_motion": "online",
        },
    )

    if time.time() - last_tick >= 1.0:
        with store._lock:
            store._state.history_time.append(time.time())
            store._state.history_cpu.append(float(cpu_usage))
            store._state.history_vel.append(float(linear_vel))
            store._state.history_base_vel.append(float(baseline_vel))
            store._state.history_latency.append(float(network_latency))
            store._state.history_battery.append(float(battery_voltage))
            store._state.history_temp.append(float(cpu_temp))
        st.session_state["demo_last_tick"] = time.time()


def _render_camera_frame(camera_jpeg: bytes, frame: np.ndarray, caption: str) -> None:
    if camera_jpeg:
        encoded = base64.b64encode(camera_jpeg).decode("ascii")
    elif frame is not None:
        # Raw fallback path when the bridge is subscribed to image_raw instead of JPEG.
        frame = frame.astype(np.uint8)
        if frame.shape[1] > 640:
            scale = 640.0 / float(frame.shape[1])
            new_size = (640, max(1, int(frame.shape[0] * scale)))
            frame = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
        success, jpeg = cv2.imencode(".jpg", frame[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not success:
            st.info("Waiting for /camera/rgb/image_raw or /camera/rgb/image_compressed...")
            return
        encoded = base64.b64encode(jpeg.tobytes()).decode("ascii")
    else:
        st.info("Waiting for /camera/rgb/image_raw or /camera/rgb/image_compressed...")
        return
    st.markdown(
        f"""
        <figure style="margin:0">
          <img
            src="data:image/jpeg;base64,{encoded}"
            alt="{caption}"
            style="width:100%;border-radius:0.5rem;display:block;"
          />
          <figcaption style="margin-top:0.35rem;color:rgba(245,247,251,0.72);font-size:0.82rem;">
            {caption}
          </figcaption>
        </figure>
        """,
        unsafe_allow_html=True,
    )


def _render_camera_stream(caption: str, camera_name: str, *, stream_port: int = 8767, height: int = 430) -> None:
    components.html(
        f"""
        <div style="margin:0">
          <div style="position:relative;border-radius:0.5rem;overflow:hidden;background:#0f172a;aspect-ratio:4/3;min-height:{height}px;">
            <img
              id="lunar-camera-stream"
              alt="{caption}"
              style="position:absolute;inset:0;width:100%;height:100%;display:block;object-fit:cover;"
            />
            <div
              id="lunar-camera-empty"
              style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:1rem;color:#dbeafe;background:#12233d;"
            >
              Waiting for {camera_name} websocket feed...
            </div>
          </div>
          <div style="margin-top:0.35rem;color:rgba(245,247,251,0.72);font-size:0.82rem;">
            {caption}
          </div>
        </div>
        <script>
          const img = document.getElementById("lunar-camera-stream");
          const empty = document.getElementById("lunar-camera-empty");

          function resolveHostContext() {{
            try {{
              if (window.parent && window.parent.location && window.parent.location.hostname) {{
                return {{
                  protocol: window.parent.location.protocol || "http:",
                  hostname: window.parent.location.hostname,
                }};
              }}
            }} catch (error) {{}}

            try {{
              if (document.referrer) {{
                const ref = new URL(document.referrer);
                if (ref.hostname) {{
                  return {{
                    protocol: ref.protocol || "http:",
                    hostname: ref.hostname,
                  }};
                }}
              }}
            }} catch (error) {{}}

            return {{
              protocol: window.location.protocol || "http:",
              hostname: window.location.hostname || "127.0.0.1",
            }};
          }}

          const hostCtx = resolveHostContext();
          const scheme = hostCtx.protocol === "https:" ? "wss" : "ws";
          const wsUrl = `${{scheme}}://${{hostCtx.hostname}}:{stream_port}/camera/ws/{camera_name}`;
          let socket = null;
          let retryId = null;
          let currentObjectUrl = null;

          function clearObjectUrl() {{
            if (currentObjectUrl) {{
              URL.revokeObjectURL(currentObjectUrl);
              currentObjectUrl = null;
            }}
          }}

          function scheduleReconnect() {{
            if (retryId !== null) {{
              window.clearTimeout(retryId);
            }}
            retryId = window.setTimeout(connect, 500);
          }}

          function connect() {{
            try {{
              socket = new WebSocket(wsUrl);
            }} catch (error) {{
              empty.style.display = "flex";
              scheduleReconnect();
              return;
            }}

            socket.binaryType = "arraybuffer";

            socket.onopen = () => {{
              empty.style.display = "flex";
            }};

            socket.onmessage = (event) => {{
              const payload = event.data instanceof Blob ? event.data : new Blob([event.data], {{ type: "image/jpeg" }});
              clearObjectUrl();
              currentObjectUrl = URL.createObjectURL(payload);
              img.src = currentObjectUrl;
              empty.style.display = "none";
            }};

            socket.onerror = () => {{
              empty.style.display = "flex";
            }};

            socket.onclose = () => {{
              empty.style.display = "flex";
              scheduleReconnect();
            }};
          }}

          window.addEventListener("beforeunload", () => {{
            if (socket) {{
              socket.close();
            }}
            clearObjectUrl();
          }});

          connect();
        </script>
        """,
        height=height,
        scrolling=False,
    )


def _clamp_percent(value: int) -> int:
    return max(0, min(100, int(value)))


def _sync_control_state(state, ros_live: bool) -> None:
    desired = {
        "teleop_throttle": int(state.teleop_throttle),
        "bucket_chain_speed": int(state.bucket_chain_speed),
        "conveyor_enabled": bool(state.conveyor_enabled),
        "camera_pan": int(state.camera_pan),
        "camera_height": int(state.camera_height),
        "bucket_pos": int(state.bucket_pos),
    }
    for key, value in desired.items():
        if (not ros_live) or key not in st.session_state:
            st.session_state[key] = value


def _apply_hold_event(node, event) -> None:
    if node is None or not isinstance(event, dict):
        return
    kind = event.get("kind")
    command = event.get("command")
    active = event.get("active")
    if isinstance(kind, str) and isinstance(command, str):
        node.handle_hold_event(kind, command, bool(active))


def _fragment(run_every=None):
    streamlit_fragment = getattr(st, "fragment", None)
    if streamlit_fragment is None:
        def decorator(func):
            return func
        return decorator
    return streamlit_fragment(run_every=run_every)


@_fragment(run_every=UI_REFRESH_SEC)
def _render_sidebar_monitor():
    state = store.get_snapshot()
    st.subheader("💻 OS Monitor")
    st.progress(state.cpu_usage / 100.0, text=f"CPU: {state.cpu_usage}%")
    st.progress(state.ram_usage / 100.0, text=f"RAM: {state.ram_usage}%")
    st.metric("CPU Temp", f"{state.cpu_temp:.1f} °C")


@_fragment(run_every=UI_REFRESH_SEC)
def _render_command_metrics():
    state = store.get_snapshot()
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Linear Vel", f"{state.linear_vel:.2f} m/s")
    c2.metric("Pos [X, Y]", f"{state.odom_x:.1f}, {state.odom_y:.1f}")
    c3.metric("Battery", f"{state.battery_voltage:.1f} V")
    c4.metric("Latency", f"{state.network_latency:.1f} ms")


def _render_camera_panel():
    st.subheader("👁️ Vision Feed")
    tab_rgb, tab_rear, tab_tracking = st.tabs(["Front D435 RGB", "Rear D435 RGB", "T265 Tracking"])
    with tab_rgb:
        _render_camera_stream("Front D435 RGB", "rgb", height=520)
    with tab_rear:
        _render_camera_stream("Rear D435 RGB", "rear", height=520)
    with tab_tracking:
        _render_camera_stream("T265 Tracking", "tracking", height=520)


@_fragment(run_every=UI_REFRESH_SEC)
def _render_command_sensors():
    st.subheader("📡 Live Sensors")
    state = store.get_snapshot()
    s1, s2 = st.columns(2)
    s1.metric("IR Left", f"{int(state.ir_left)}")
    s2.metric("IR Right", f"{int(state.ir_right)}")
    e1, e2 = st.columns(2)
    e1.metric("Encoder Left", f"{int(state.encoder_left)}")
    e2.metric("Encoder Right", f"{int(state.encoder_right)}")


@_fragment(run_every=UI_REFRESH_SEC)
def _render_map_panel():
    state = store.get_snapshot()
    st.subheader("🗺️ Mapping")
    if state.map_data is not None:
        fig = px.imshow(state.map_data, color_continuous_scale='gray_r', origin='lower')
        fig.update_layout(margin=dict(l=0, r=0, t=0, b=0), height=350)
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.warning("No Map Data (/map)")


@_fragment(run_every=UI_REFRESH_SEC)
def _render_command_controls():
    node = get_node() if st.session_state.get('ros_bridge', False) else None
    ros_live = node is not None and st.session_state.get('ros_bridge', False)

    st.subheader("🕹️ Controls")
    if not ros_live:
        st.warning("Live ROS telemetry is disabled. Teleop controls are visible but inactive.")

    st.markdown('<div class="compact-card"><h4>Drive</h4></div>', unsafe_allow_html=True)
    td1, td2 = st.columns(2)
    if td1.button("-10", key="teleop_throttle_down", use_container_width=True, disabled=not ros_live):
        st.session_state["teleop_throttle"] = _clamp_percent(st.session_state["teleop_throttle"] - 10)
    if td2.button("+10", key="teleop_throttle_up", use_container_width=True, disabled=not ros_live):
        st.session_state["teleop_throttle"] = _clamp_percent(st.session_state["teleop_throttle"] + 10)
    throttle = st.slider(
        "Throttle",
        0,
        100,
        key="teleop_throttle",
        disabled=not ros_live,
    )
    if ros_live and int(throttle) != int(store.get_snapshot().teleop_throttle):
        store.update(teleop_throttle=int(throttle))
    drive_event = render_hold_controls("drive", disabled=not ros_live, key="teleop_drive_hold")
    _apply_hold_event(node, drive_event)
    drive_state = store.get_snapshot()
    st.caption(
        f"Throttle {int(st.session_state['teleop_throttle'])} | "
        f"Drive {drive_state.active_drive_cmd or 'stopped'}"
    )

    compact_left, compact_right = st.columns(2)
    with compact_left:
        st.markdown('<div class="compact-card"><h4>Mining</h4></div>', unsafe_allow_html=True)
        chain_speed = st.slider(
            "Chain Speed",
            0,
            100,
            key="bucket_chain_speed",
            disabled=not ros_live,
        )
        if ros_live and int(chain_speed) != int(store.get_snapshot().bucket_chain_speed):
            store.update(bucket_chain_speed=int(chain_speed))
        bucket_event = render_hold_controls("bucket_vel", disabled=not ros_live, key="teleop_bucket_hold")
        _apply_hold_event(node, bucket_event)
        conveyor_enabled = st.toggle(
            "Conveyor",
            key="conveyor_enabled",
            disabled=not ros_live,
        )
        if ros_live and bool(conveyor_enabled) != bool(store.get_snapshot().conveyor_enabled):
            node.publish_conveyor(bool(conveyor_enabled))
        bucket_pos = st.slider(
            "Bucket Pos",
            0,
            100,
            key="bucket_pos",
            disabled=not ros_live,
        )
        if ros_live and int(bucket_pos) != int(store.get_snapshot().bucket_pos):
            node.publish_bucket_pos(int(bucket_pos))
        mining_state = store.get_snapshot()
        st.caption(
            f"Chain {mining_state.active_bucket_cmd or 'stopped'} | "
            f"Conveyor {'on' if mining_state.conveyor_enabled else 'off'}"
        )

    with compact_right:
        st.markdown('<div class="compact-card"><h4>Camera</h4></div>', unsafe_allow_html=True)
        camera_pan = st.slider(
            "Pan",
            0,
            180,
            key="camera_pan",
            disabled=not ros_live,
        )
        if ros_live and int(camera_pan) != int(store.get_snapshot().camera_pan):
            node.publish_pan(int(camera_pan))
        camera_height = st.slider(
            "Cam Height",
            0,
            100,
            key="camera_height",
            disabled=not ros_live,
        )
        if ros_live and int(camera_height) != int(store.get_snapshot().camera_height):
            node.publish_cam_height(int(camera_height))
        camera_state = store.get_snapshot()
        st.caption(
            f"Height {int(st.session_state['camera_height'])} | "
            f"Pan {int(camera_state.camera_pan)}"
        )

    st.markdown('<div class="compact-card"><h4>Navigation & Macros</h4></div>', unsafe_allow_html=True)
    cx, cy, cgo = st.columns([1, 1, 1.1])
    tx = cx.number_input("X", value=0.0, disabled=not ros_live)
    ty = cy.number_input("Y", value=0.0, disabled=not ros_live)
    if cgo.button("🚀 GO TO", use_container_width=True, disabled=not ros_live):
        node.publish_goal(tx, ty)

    m1, m2 = st.columns(2)
    if m1.button("🏁 Home", use_container_width=True, disabled=not ros_live):
        node.trigger_macro("home")
    if m2.button("⛏️ Dig", use_container_width=True, disabled=not ros_live):
        node.trigger_macro("dig")

    if st.button(
        "🔴 EMERGENCY STOP",
        type="primary",
        use_container_width=True,
        disabled=not ros_live,
        key="dashboard_emergency_stop",
    ):
        node.stop_all_actuators()
        st.session_state["conveyor_enabled"] = False
        st.toast("All motion commands stopped.")
        st.rerun()


@_fragment(run_every=UI_REFRESH_SEC)
def _render_analytics_live():
    state = store.get_snapshot()
    col_pulse, col_radar = st.columns([2, 1])

    with col_pulse:
        st.subheader("📈 System Vital Trends")
        if len(state.history_time) > 2:
            df_hist = pd.DataFrame({
                "Time": list(state.history_time),
                "CPU Load (%)": list(state.history_cpu),
                "Temp (°C)": list(state.history_temp),
                "Battery (V)": list(state.history_battery),
            })
            df_hist["Time"] -= df_hist["Time"].iloc[0]
            st.line_chart(df_hist, x="Time", y=["CPU Load (%)", "Temp (°C)", "Battery (V)"], height=300)

        st.subheader("📉 Odometry Divergence (V-SLAM vs Baseline)")
        if len(state.history_time) > 2:
            df_odom = pd.DataFrame({
                "Time": list(state.history_time),
                "V-SLAM (T265)": list(state.history_vel),
                "Baseline (Cmd)": list(state.history_base_vel),
            })
            df_odom["Time"] -= df_odom["Time"].iloc[0]
            st.line_chart(df_odom, x="Time", y=["V-SLAM (T265)", "Baseline (Cmd)"], height=300)

    with col_radar:
        st.subheader("🦇 IR Proximity Radar")
        fig_radar = go.Figure(go.Scatterpolar(
            r=[state.ir_left, state.ir_right],
            theta=[135, 45],
            mode='markers+lines',
            marker=dict(size=15, color="red"),
            fill='toself',
            name="Proximity"
        ))
        fig_radar.update_layout(
            polar=dict(
                radialaxis=dict(visible=True, range=[0, 1024]),
                angularaxis=dict(tickvals=[0, 45, 90, 135, 180], rotation=90, direction="counterclockwise")
            ),
            showlegend=False,
            height=350,
            margin=dict(l=20, r=20, t=20, b=20)
        )
        st.plotly_chart(fig_radar, use_container_width=True)


@_fragment(run_every=UI_REFRESH_SEC)
def _render_logs_live():
    state = store.get_snapshot()
    st.code("\n".join(state.recent_logs), language="bash")


@_fragment(run_every=UI_REFRESH_SEC)
def _render_hardware_live(repo_root: Path):
    render_hardware_panel(repo_root)


@_fragment(run_every=UI_REFRESH_SEC)
def _render_hardware_controls():
    st.divider()
    st.subheader("🛠️ Node Control")
    cs1, cs2, cs3 = st.columns(3)
    if cs1.button("🔄 Restart Mapper"):
        subprocess.Popen("ros2 node kill /mapper", shell=True)
    if cs2.button("🔄 Restart Controller"):
        subprocess.Popen("ros2 node kill /motion_controller", shell=True)
    if cs3.button("🔄 Restart Bridge"):
        subprocess.Popen("ros2 node kill /lunar_dashboard_bridge", shell=True)


@_fragment(run_every=UI_REFRESH_SEC)
def _render_data_status_live():
    state = store.get_snapshot()
    if state.is_recording:
        st.warning(f"Recording: {state.bag_filename}")
    else:
        st.caption("Recorder idle.")

st.set_page_config(page_title="Lunar Mission Control", page_icon="🌔", layout="wide")
st.markdown(
    """
    <style>
    div[data-testid="stButton"] button {
        min-height: 2.3rem;
        padding: 0.35rem 0.6rem;
    }
    div[data-testid="stSlider"] {
        margin-bottom: 0.15rem;
    }
    div[data-testid="stToggle"] {
        margin-top: 0.1rem;
        margin-bottom: 0.2rem;
    }
    div[data-testid="stNumberInput"] {
        margin-bottom: 0.15rem;
    }
    .compact-card {
        border: 1px solid rgba(255, 255, 255, 0.08);
        border-radius: 14px;
        padding: 0.7rem 0.8rem 0.55rem;
        background: rgba(255, 255, 255, 0.02);
        margin-bottom: 0.6rem;
    }
    .compact-card h4 {
        margin: 0 0 0.35rem 0;
        font-size: 0.92rem;
        font-weight: 700;
        letter-spacing: 0.02em;
    }
    .compact-note {
        margin: 0.2rem 0 0.35rem;
        color: rgba(245, 247, 251, 0.74);
        font-size: 0.8rem;
        line-height: 1.3;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

# Initialize bridge only once
if 'ros_bridge' not in st.session_state:
    st.session_state['ros_bridge'] = False
    st.session_state['ros_bridge_error'] = None
    if BRIDGE_IMPORT_ERROR is not None:
        st.session_state['ros_bridge_error'] = str(BRIDGE_IMPORT_ERROR)
    else:
        try:
            start_ros_thread()
            st.session_state['ros_bridge'] = True
        except Exception as e:
            st.session_state['ros_bridge_error'] = str(e)

st.title("🌔 Lunar Mission Control")

if not st.session_state.get('ros_bridge', False):
    reason = st.session_state.get('ros_bridge_error') or "ROS bridge unavailable"
    st.error(f"Dashboard startup failed: {reason}")
    st.stop()

node = get_node() if st.session_state.get('ros_bridge', False) else None
ros_live = node is not None and st.session_state.get('ros_bridge', False)
state = store.get_snapshot()
_sync_control_state(state, ros_live)

# --- Sidebar Configuration ---
with st.sidebar:
    st.header("⚙️ Global Settings")
    with st.expander("🎯 PID Tuning", expanded=False):
        st.write("Target: Drive Controller")
        np_p = st.slider("Kp (Proportional)", 0.0, 10.0, float(state.kp), 0.1)
        ni_i = st.slider("Ki (Integral)", 0.0, 5.0, float(state.ki), 0.01)
        nd_d = st.slider("Kd (Derivative)", 0.0, 5.0, float(state.kd), 0.01)
        if st.button("Apply Gains", use_container_width=True):
            bridge = get_node()
            if bridge:
                bridge.publish_pid(np_p, ni_i, nd_d)
            st.toast("PID Gains Updated!")
    
    st.divider()
    _render_sidebar_monitor()

tab_cmd, tab_tele, tab_hw, tab_data = st.tabs(["🎮 Command Center", "📊 Analytics & Pulse", "🔍 Hardware", "💾 Data Ops"])

# --- Tab 1: Command Center ---
with tab_cmd:
    _render_command_metrics()

    col_vis, col_sensors = st.columns([2.25, 1.0])
    
    with col_vis:
        _render_camera_panel()
        _render_map_panel()

    with col_sensors:
        _render_command_sensors()

# --- Tab 2: Analytics & Pulse ---
with tab_tele:
    _render_analytics_live()

# --- Tab 3: Hardware ---
with tab_hw:
    _render_hardware_live(repo_root)
    _render_hardware_controls()

# --- Tab 4: Data Operations ---
with tab_data:
    st.subheader("💾 Data Management")
    c_bag, c_map = st.columns(2)
    with c_bag:
        fname = st.text_input("Bag Name", "mission_data")
        if not state.is_recording:
            if st.button("🔴 Start Recording", use_container_width=True):
                bridge = get_node()
                if bridge:
                    bridge.toggle_recording(fname)
        else:
            if st.button("⏹️ Stop Recording", type="primary", use_container_width=True):
                bridge = get_node()
                if bridge:
                    bridge.toggle_recording(fname)
            st.warning(f"Recording: {state.bag_filename}")
    with c_map:
        mname = st.text_input("Map Name", "lunar_v1")
        if st.button("💾 Save Map", use_container_width=True):
            bridge = get_node()
            if bridge:
                bridge.save_map(mname)
            st.success("Map saving triggered.")

    st.divider()
    _render_data_status_live()
    st.divider()
    st.subheader("📜 System Logs")
    _render_logs_live()