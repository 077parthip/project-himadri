"""
PROJECT HIMADRI - Bio-Aware Thermal Cold Storage  |  SIH 2026 (SIH26005)
Live IoT dashboard (Streamlit + Plotly)

Run:   streamlit run app.py

DATA SOURCES
  * Simulation (default): generates MQTT-like telemetry so the jury can see the
    full story - normal storage, rot in Zone C, damper opening - with no hardware.
  * Live MQTT (optional): subscribes to  himadri/zone/<A-D>/telemetry  with JSON
      {"temp": 5.2, "hum": 88.1, "co2": 640, "damper": false}
    and himadri/pcm  with  {"temp": -1.5}
"""

import json
import math
import queue
import random
import time
from collections import deque
from datetime import datetime

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

# ----------------------------------------------------------------------------
# CONSTANTS (mirror the firmware thresholds so both sides agree)
# ----------------------------------------------------------------------------
ZONES = ["A", "B", "C", "D"]
TEMP_LIMIT_C = 8.0
CO2_LIMIT_PPM = 1000
PCM_FROZEN_C, PCM_WARM_C = -2.0, 6.0
HISTORY_LEN = 120  # samples kept on the charts

st.set_page_config(page_title="Himadri | Bio-Aware Cold Storage",
                   page_icon="🧊", layout="wide")

# ----------------------------------------------------------------------------
# DARK-MODE STYLING (Streamlit's theme is set in .streamlit/config.toml; this
# CSS adds the card look and the zone status tiles)
# ----------------------------------------------------------------------------
st.markdown("""
<style>
.block-container {padding-top: 1.2rem;}
.zone-card {border-radius: 14px; padding: 16px 18px; border: 1px solid #263043;
            background: #131a26; height: 100%;}
.zone-ok   {border-left: 6px solid #22c55e;}
.zone-warn {border-left: 6px solid #ef4444; background: #2a1517;
            animation: pulse 1.4s infinite;}
.zone-title {font-size: 1.05rem; font-weight: 700; color: #e5e7eb;}
.zone-badge-ok   {color:#22c55e; font-weight:600;}
.zone-badge-warn {color:#ef4444; font-weight:700;}
.zone-metric {color:#9ca3af; font-size:0.85rem; margin-top:4px;}
@keyframes pulse {0%{box-shadow:0 0 0 0 rgba(239,68,68,.5);}
                  70%{box-shadow:0 0 0 12px rgba(239,68,68,0);}
                  100%{box-shadow:0 0 0 0 rgba(239,68,68,0);}}
</style>
""", unsafe_allow_html=True)


# ----------------------------------------------------------------------------
# OPTIONAL LIVE MQTT SUBSCRIBER (runs once, in a background thread)
# ----------------------------------------------------------------------------
@st.cache_resource
def start_mqtt(host: str, port: int):
    """Connect to the broker and push messages into a thread-safe queue."""
    import paho.mqtt.client as mqtt  # imported lazily: not needed in demo mode

    q: "queue.Queue[tuple[str, dict]]" = queue.Queue()

    def on_connect(client, userdata, flags, reason_code, properties=None):
        client.subscribe("himadri/#")

    def on_message(client, userdata, msg):
        try:
            q.put((msg.topic, json.loads(msg.payload)))
        except json.JSONDecodeError:
            pass  # ignore malformed packets

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect, client.on_message = on_connect, on_message
    client.connect(host, port, keepalive=30)
    client.loop_start()
    return q


# ----------------------------------------------------------------------------
# SIMULATOR - produces the same shape of data the ESP32 would publish
# ----------------------------------------------------------------------------
def simulate_step(state: dict, rot_in_zone_c: bool) -> dict:
    """Advance the fake world by one tick and return {zone: reading}."""
    state["t"] += 1
    t = state["t"]

    # Day/night cycle (compressed: 80 ticks = 1 "day")
    day = math.sin(2 * math.pi * t / 80) > 0
    # Thermal battery: charges (freezes) by day, slowly melts at night
    state["pcm"] += -0.12 if day else +0.05
    state["pcm"] = max(-3.0, min(7.0, state["pcm"]))
    state["day"] = day

    readings = {}
    for i, z in enumerate(ZONES):
        temp = 5.5 + 0.6 * math.sin(t / 15 + i) + random.uniform(-0.15, 0.15)
        hum = 88 + random.uniform(-1.5, 1.5)
        co2 = 620 + 40 * i + random.uniform(-25, 25)

        if z == "C" and rot_in_zone_c and t > 25:      # rot begins after ~25 ticks
            # rot_level = extra CO2 from respiring/rotting produce. It grows
            # while the damper is closed and is flushed while it is open.
            venting = state["damper"].get(z, False)
            state["rot"] = max(0.0, state["rot"] + (-70 if venting else 22))
            co2 += state["rot"]
            temp += min(3.5, state["rot"] * 0.004)       # microbial heat

        # Same rule as firmware, with hysteresis (latched until safe again)
        was_open = state["damper"].get(z, False)
        bad = co2 > CO2_LIMIT_PPM or temp > TEMP_LIMIT_C
        clear = co2 < CO2_LIMIT_PPM - 100 and temp < TEMP_LIMIT_C - 1
        state["damper"][z] = bad or (was_open and not clear)

        readings[z] = dict(temp=temp, hum=hum, co2=max(400, co2),
                           damper=state["damper"][z])
    return readings


def init_state():
    if "sim" not in st.session_state:
        st.session_state.sim = {"t": 0, "pcm": 4.0, "day": True,
                                "damper": {}, "rot": 0.0}
        st.session_state.history = {z: deque(maxlen=HISTORY_LEN) for z in ZONES}
        st.session_state.latest = {}
        st.session_state.pcm_temp = 4.0
        st.session_state.day = True


def ingest_mqtt(q):
    """Drain queued MQTT messages into session state."""
    while not q.empty():
        topic, data = q.get()
        parts = topic.split("/")
        if len(parts) == 4 and parts[1] == "zone" and parts[2] in ZONES:
            z = parts[2]
            st.session_state.latest[z] = data
            st.session_state.history[z].append(
                {"time": datetime.now(), **{k: data.get(k) for k in ("temp", "hum", "co2")}})
        elif topic == "himadri/pcm":
            st.session_state.pcm_temp = data.get("temp", st.session_state.pcm_temp)


# ----------------------------------------------------------------------------
# UI HELPERS
# ----------------------------------------------------------------------------
def freeze_percent(pcm_temp: float) -> float:
    """Map wall temp (6 C -> 0 %, -2 C -> 100 %) to a 'charge' percentage."""
    pct = (PCM_WARM_C - pcm_temp) / (PCM_WARM_C - PCM_FROZEN_C) * 100
    return max(0.0, min(100.0, pct))


def gauge(pct: float) -> go.Figure:
    fig = go.Figure(go.Indicator(
        mode="gauge+number",
        value=pct,
        number={"suffix": "%", "font": {"size": 44}},
        title={"text": "Thermal Battery Freeze Status"},
        gauge={"axis": {"range": [0, 100]},
               "bar": {"color": "#38bdf8"},
               "bgcolor": "#131a26",
               "steps": [{"range": [0, 30], "color": "#3b1d1d"},
                         {"range": [30, 70], "color": "#3a3218"},
                         {"range": [70, 100], "color": "#123524"}],
               "threshold": {"line": {"color": "#22c55e", "width": 4},
                             "value": 90}}))
    fig.update_layout(template="plotly_dark", height=300,
                      margin=dict(l=20, r=20, t=60, b=10),
                      paper_bgcolor="rgba(0,0,0,0)")
    return fig


def line_chart(df: pd.DataFrame, y: str, title: str, unit: str,
               limit: float | None = None) -> go.Figure:
    fig = go.Figure()
    for z in ZONES:
        d = df[df.zone == z]
        fig.add_trace(go.Scatter(x=d.time, y=d[y], name=f"Zone {z}", mode="lines"))
    if limit is not None:
        fig.add_hline(y=limit, line_dash="dash", line_color="#ef4444",
                      annotation_text=f"limit {limit}{unit}")
    fig.update_layout(template="plotly_dark", title=title, height=320,
                      margin=dict(l=10, r=10, t=50, b=10),
                      paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(0,0,0,0)",
                      legend=dict(orientation="h", y=-0.2))
    return fig


def zone_card(z: str, r: dict) -> str:
    warn = r["damper"]
    cls = "zone-warn" if warn else "zone-ok"
    badge = ('<span class="zone-badge-warn">🔴 Rot Warning! Damper Open</span>' if warn
             else '<span class="zone-badge-ok">🟢 Healthy · Damper Closed</span>')
    return (f'<div class="zone-card {cls}"><div class="zone-title">Zone {z}</div>{badge}'
            f'<div class="zone-metric">🌡 {r["temp"]:.1f} °C &nbsp; 💧 {r["hum"]:.0f} %</div>'
            f'<div class="zone-metric">CO₂ {r["co2"]:.0f} ppm</div></div>')


# ----------------------------------------------------------------------------
# SIDEBAR
# ----------------------------------------------------------------------------
init_state()
with st.sidebar:
    st.title("🧊 Himadri")
    st.caption("Bio-Aware Thermal Cold Storage · SIH26005")
    mode = st.radio("Data source", ["Simulation", "Live MQTT"])
    rot_sim = st.checkbox("Simulate rot in Zone C", value=True)
    if mode == "Live MQTT":
        host = st.text_input("Broker host", "localhost")
        port = st.number_input("Port", 1, 65535, 1883)
    if st.button("Reset demo"):
        for k in ("sim", "history", "latest"):
            st.session_state.pop(k, None)
        st.rerun()

st.title("Project Himadri · Live Storage Monitor")


# ----------------------------------------------------------------------------
# MAIN PANEL - a fragment re-runs every 2 s without reloading the whole page
# ----------------------------------------------------------------------------
@st.fragment(run_every=2)
def live_panel():
    if mode == "Live MQTT":
        ingest_mqtt(start_mqtt(host, int(port)))
    else:
        readings = simulate_step(st.session_state.sim, rot_sim)
        st.session_state.latest = readings
        st.session_state.pcm_temp = st.session_state.sim["pcm"]
        st.session_state.day = st.session_state.sim["day"]
        now = datetime.now()
        for z, r in readings.items():
            st.session_state.history[z].append(
                {"time": now, "temp": r["temp"], "hum": r["hum"], "co2": r["co2"]})

    latest = st.session_state.latest
    if not latest:
        st.info("Waiting for telemetry…")
        return

    # --- top row: gauge + system status --------------------------------------
    c1, c2 = st.columns([1, 1.6])
    with c1:
        st.plotly_chart(gauge(freeze_percent(st.session_state.pcm_temp)),
                        use_container_width=True)
    with c2:
        day = st.session_state.day
        alerts = [z for z, r in latest.items() if r["damper"]]
        m1, m2, m3 = st.columns(3)
        m1.metric("Mode", "☀️ Day" if day else "🌙 Night")
        m2.metric("Compressor", "ON (solar)" if day and freeze_percent(
            st.session_state.pcm_temp) < 100 else "OFF")
        m3.metric("Active alerts", len(alerts))
        avg_t = sum(r["temp"] for r in latest.values()) / len(latest)
        st.metric("Average room temp", f"{avg_t:.1f} °C",
                  delta=f"{avg_t - 6:+.1f} vs 6 °C target", delta_color="inverse")
        if alerts:
            st.error(f"SMS dispatched to farmer · Zone(s) {', '.join(alerts)}")

    # --- zone status tiles ----------------------------------------------------
    st.subheader("Storage zones")
    cols = st.columns(4)
    for col, z in zip(cols, ZONES):
        if z in latest:
            col.markdown(zone_card(z, latest[z]), unsafe_allow_html=True)

    # --- charts ---------------------------------------------------------------
    rows = [{"zone": z, **row} for z in ZONES for row in st.session_state.history[z]]
    df = pd.DataFrame(rows)
    if not df.empty:
        st.plotly_chart(line_chart(df, "co2", "CO₂ per zone (ppm)", " ppm", CO2_LIMIT_PPM),
                        use_container_width=True)
        a, b = st.columns(2)
        a.plotly_chart(line_chart(df, "temp", "Temperature (°C)", " °C", TEMP_LIMIT_C),
                       use_container_width=True)
        b.plotly_chart(line_chart(df, "hum", "Relative humidity (%)", " %"),
                       use_container_width=True)


live_panel()
