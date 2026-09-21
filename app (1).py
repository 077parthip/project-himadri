"""
PROJECT HIMADRI - Bio-Aware Thermal Cold Storage  |  SIH 2026 (SIH26005)
Live IoT dashboard (Streamlit + Plotly) - v2 with energy-optimised fan control

Run:   streamlit run app.py        (or: python -m streamlit run app.py)

DATA SOURCES
  * Simulation (default): a small closed-loop plant model, so the jury can see the whole
    story with no hardware - normal storage, rot in Zone C, damper opening, fans throttling.
  * Live MQTT (optional): subscribes to
        himadri/zone/<A-D>/telemetry  {"temp":5.2,"hum":88,"co2":640,"damper":false,
                                       "fan":42,"rpm":1260}
        himadri/pcm                   {"temp":-1.5}
        himadri/energy                {"power_w":1.9,"energy_wh":0.84,"baseline_wh":2.1}

FAN CONTROL (same algorithm as firmware/main.cpp -> fanController())
  * PI on temperature (setpoint 6 C, +/-0.3 C deadband, anti-windup)
  * Humidity trim: RH < 85 % -> slower, RH > 95 % -> faster
  * 25 % floor, 50 % cap at night, 10 %/s slew limit
  * Damper open (rot / warm alarm) -> 100 % to flush the zone
  * Fan power ~ speed^3, compared with a fixed-100 % baseline -> "energy saved"
"""

import json
import math
import queue
import random
from collections import deque
from datetime import datetime

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

# ----------------------------------------------------------------------------
# CONSTANTS (mirror the firmware so both sides agree)
# ----------------------------------------------------------------------------
ZONES = ["A", "B", "C", "D"]
TEMP_LIMIT_C = 8.0
CO2_LIMIT_PPM = 1000
PCM_FROZEN_C, PCM_WARM_C = -2.0, 6.0
HISTORY_LEN = 180          # samples kept on the charts
DAY_LEN = 300              # simulated day length in ticks (1 tick = 1 s)

FAN = dict(SP=6.0, DEADBAND=0.3, KP=25.0, KI=1.5, BIAS=30.0, INT_MIN=-20.0, INT_MAX=40.0,
           MIN=25.0, MAX_DAY=100.0, MAX_NIGHT=50.0, SLEW=10.0, RH_DRY=85.0, RH_WET=95.0,
           RATED_W=2.4, MAX_RPM=3000.0, FAIL_RPM=200.0)

st.set_page_config(page_title="Himadri | Bio-Aware Cold Storage",
                   page_icon="🧊", layout="wide")

# ----------------------------------------------------------------------------
# DARK-MODE STYLING (theme in .streamlit/config.toml; CSS adds the card look)
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
.zone-fan {color:#38bdf8; font-size:0.85rem; margin-top:6px; font-weight:600;}
.zone-fault {color:#ef4444; font-weight:700; margin-top:4px; font-size:0.85rem;}
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
# FAN CONTROLLER  (same algorithm as the ESP32 firmware)
# ----------------------------------------------------------------------------
def fan_controller(S: dict, z: str, temp: float, rh: float, day: bool,
                   vent_override: bool, dt: float) -> float:
    """Return the commanded fan speed (%) for one zone."""
    if vent_override:                       # rot / warm alarm -> flush at full speed
        return 100.0
    err = temp - FAN["SP"]
    if abs(err) < FAN["DEADBAND"]:          # deadband: no action near the setpoint
        err = 0.0
    # integral term with anti-windup clamp
    S["integ"][z] = max(FAN["INT_MIN"], min(FAN["INT_MAX"], S["integ"][z] + err * dt))
    target = FAN["BIAS"] + FAN["KP"] * err + FAN["KI"] * S["integ"][z]
    if rh < FAN["RH_DRY"]:                  # humidity trim: dry air -> protect produce
        target *= 0.7
    elif rh > FAN["RH_WET"]:                # too humid -> avoid condensation / mould
        target += 15.0
    target = max(FAN["MIN"], min(FAN["MAX_DAY"] if day else FAN["MAX_NIGHT"], target))
    step = FAN["SLEW"] * dt                 # slew-rate limit (no hunting / noise)
    return S["fan"][z] + max(-step, min(step, target - S["fan"][z]))


# ----------------------------------------------------------------------------
# SIMULATOR - closed-loop plant, produces the same data the ESP32 would publish
# ----------------------------------------------------------------------------
def new_sim() -> dict:
    return {"t": 0, "pcm": 3.0, "day": False, "comp": False, "rot": 0.0,
            "damper": {z: False for z in ZONES}, "cause": {z: "" for z in ZONES},
            "fan": {z: FAN["MIN"] for z in ZONES}, "integ": {z: 0.0 for z in ZONES},
            "simT": {z: 6.0 for z in ZONES},
            "power_w": 0.0, "energy_wh": 0.0, "baseline_wh": 0.0}


def simulate_step(S: dict, rot_on: bool, dry: bool, fan_fault_b: bool) -> dict:
    """Advance the fake world by one second and return {zone: reading}."""
    S["t"] += 1
    t = S["t"]
    phase = (t % DAY_LEN) / DAY_LEN

    # Solar array: sine-shaped daylight in the first half of the cycle. Day = > 12 V.
    pv = 1 + 21 * math.sin(2 * math.pi * phase) if phase < 0.5 else 0.3 + random.uniform(0, .2)
    S["day"] = pv > 12
    S["pv"] = pv

    # Compressor only in daylight and only until the glycol wall is frozen (<= -2 C)
    S["comp"] = S["day"] and S["pcm"] > PCM_FROZEN_C
    S["pcm"] += -0.06 if S["comp"] else +0.012
    S["pcm"] = max(-4.0, min(8.0, S["pcm"]))

    # Rot in Zone C: extra CO2 grows until venting flushes it out
    if rot_on and t > 30:
        S["rot"] = max(0.0, min(1400.0, S["rot"] + (-14 if S["damper"]["C"] else 8)))
    elif not rot_on:
        S["rot"] = max(0.0, S["rot"] - 40)

    readings, power = {}, 0.0
    for i, z in enumerate(ZONES):
        rot = S["rot"] if z == "C" else 0.0
        fan0 = S["fan"][z]

        # First-order thermal plant: heat leaks in from 12 C ambient; air moved by the
        # fan over the cold glycol wall removes it. More fan = more cooling (and more power).
        S["simT"][z] += (0.010 * (12 - S["simT"][z])
                         - 0.021 * (fan0 / 100) * (S["simT"][z] - S["pcm"])
                         + (0.02 if z == "C" and rot_on and t > 60 else 0.0))
        temp = S["simT"][z] + random.uniform(-.08, .08)
        hum = 90 - 0.03 * fan0 - (9 if dry else 0) + random.uniform(-1, 1)
        co2 = 600 + 40 * i + random.uniform(-15, 15) + rot

        # Bio-aware damper logic (latched with hysteresis) - identical to firmware
        bad = co2 > CO2_LIMIT_PPM or temp > TEMP_LIMIT_C
        clear = co2 < CO2_LIMIT_PPM - 100 and temp < TEMP_LIMIT_C - 1
        if bad and not S["damper"][z]:
            S["cause"][z] = "ROT" if co2 > CO2_LIMIT_PPM else "WARM"
        S["damper"][z] = bad or (S["damper"][z] and not clear)

        # Energy-optimised fan speed
        fan = fan_controller(S, z, temp, hum, S["day"], S["damper"][z], 1.0)
        S["fan"][z] = fan
        rpm = FAN["MAX_RPM"] * fan / 100 * (1 + random.uniform(-.015, .015)) if fan > 1 else 0.0
        if fan_fault_b and z == "B":         # demo: seized fan -> no tach pulses
            rpm = 0.0
        power += FAN["RATED_W"] * (fan / 100) ** 3        # affinity law: P ~ speed^3

        readings[z] = dict(temp=temp, hum=hum, co2=max(400, co2),
                           damper=S["damper"][z], cause=S["cause"][z], fan=fan, rpm=rpm)

    S["power_w"] = power
    S["energy_wh"] += power / 3600
    S["baseline_wh"] += len(ZONES) * FAN["RATED_W"] / 3600   # fixed-100 % fans, always on
    return readings


def init_state():
    if "sim" not in st.session_state:
        st.session_state.sim = new_sim()
        st.session_state.history = {z: deque(maxlen=HISTORY_LEN) for z in ZONES}
        st.session_state.latest = {}
        st.session_state.pcm_temp = 3.0
        st.session_state.day = False
        st.session_state.comp = False
        st.session_state.energy = {"power_w": 0.0, "energy_wh": 0.0, "baseline_wh": 0.0}


def ingest_mqtt(q):
    """Drain queued MQTT messages into session state."""
    while not q.empty():
        topic, data = q.get()
        parts = topic.split("/")
        if len(parts) == 4 and parts[1] == "zone" and parts[2] in ZONES:
            z = parts[2]
            st.session_state.latest[z] = data
            st.session_state.history[z].append(
                {"time": datetime.now(),
                 **{k: data.get(k) for k in ("temp", "hum", "co2", "fan", "rpm")}})
        elif topic == "himadri/pcm":
            st.session_state.pcm_temp = data.get("temp", st.session_state.pcm_temp)
        elif topic == "himadri/energy":
            st.session_state.energy.update(data)


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
               "threshold": {"line": {"color": "#22c55e", "width": 4}, "value": 90}}))
    fig.update_layout(template="plotly_dark", height=300,
                      margin=dict(l=20, r=20, t=60, b=10),
                      paper_bgcolor="rgba(0,0,0,0)")
    return fig


def line_chart(df: pd.DataFrame, y: str, title: str, unit: str,
               limit: float | None = None, yrange=None) -> go.Figure:
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
    if yrange:
        fig.update_yaxes(range=yrange)
    return fig


def zone_card(z: str, r: dict) -> str:
    warn = r["damper"]
    cls = "zone-warn" if warn else "zone-ok"
    if warn:
        msg = ("Rot Warning! Damper Open" if r.get("cause", "ROT") == "ROT"
               else "Temp High! Damper Open")
        badge = f'<span class="zone-badge-warn">🔴 {msg}</span>'
    else:
        badge = '<span class="zone-badge-ok">🟢 Healthy · Damper Closed</span>'
    fan, rpm = r.get("fan"), r.get("rpm")
    fan_line = ""
    if fan is not None:
        fan_line = f'<div class="zone-fan">🌀 Fan {fan:.0f} % · {rpm or 0:.0f} rpm</div>'
        if fan >= 30 and (rpm or 0) < FAN["FAIL_RPM"]:      # commanded but not spinning
            fan_line += '<div class="zone-fault">⚠ FAN FAULT (no tach)</div>'
    return (f'<div class="zone-card {cls}"><div class="zone-title">Zone {z}</div>{badge}'
            f'<div class="zone-metric">🌡 {r["temp"]:.1f} °C &nbsp; 💧 {r["hum"]:.0f} %</div>'
            f'<div class="zone-metric">CO₂ {r["co2"]:.0f} ppm</div>{fan_line}</div>')


# ----------------------------------------------------------------------------
# SIDEBAR
# ----------------------------------------------------------------------------
init_state()
with st.sidebar:
    st.title("🧊 Himadri")
    st.caption("Bio-Aware Thermal Cold Storage · SIH26005")
    mode = st.radio("Data source", ["Simulation", "Live MQTT"])
    if mode == "Simulation":
        rot_sim = st.checkbox("Simulate rot in Zone C", value=True)
        dry_sim = st.checkbox("Simulate dry air (RH < 85 %)", value=False,
                              help="Humidity trim slows the fans to protect produce")
        fault_sim = st.checkbox("Simulate fan fault in Zone B", value=False,
                                help="Tach shows 0 rpm while the fan is commanded")
    else:
        rot_sim = dry_sim = fault_sim = False
        host = st.text_input("Broker host", "localhost")
        port = st.number_input("Port", 1, 65535, 1883)
    if st.button("Reset demo"):
        for k in ("sim", "history", "latest", "energy"):
            st.session_state.pop(k, None)
        st.rerun()

st.title("Project Himadri · Live Storage Monitor")


# ----------------------------------------------------------------------------
# MAIN PANEL - a fragment re-runs every second without reloading the whole page
# ----------------------------------------------------------------------------
@st.fragment(run_every=1)
def live_panel():
    if mode == "Live MQTT":
        ingest_mqtt(start_mqtt(host, int(port)))
    else:
        S = st.session_state.sim
        readings = simulate_step(S, rot_sim, dry_sim, fault_sim)
        st.session_state.latest = readings
        st.session_state.pcm_temp = S["pcm"]
        st.session_state.day = S["day"]
        st.session_state.comp = S["comp"]
        st.session_state.energy = {"power_w": S["power_w"], "energy_wh": S["energy_wh"],
                                   "baseline_wh": S["baseline_wh"]}
        now = datetime.now()
        for z, r in readings.items():
            st.session_state.history[z].append(
                {"time": now, "temp": r["temp"], "hum": r["hum"], "co2": r["co2"],
                 "fan": r["fan"], "rpm": r["rpm"]})

    latest = st.session_state.latest
    if not latest:
        st.info("Waiting for telemetry…")
        return

    # --- top row: gauge + system status --------------------------------------
    c1, c2 = st.columns([1, 1.6])
    with c1:
        st.plotly_chart(gauge(freeze_percent(st.session_state.pcm_temp)),
                        width="stretch")
    with c2:
        day = st.session_state.day
        alerts = [z for z, r in latest.items() if r["damper"]]
        faults = [z for z, r in latest.items()
                  if r.get("fan") is not None and r["fan"] >= 30
                  and (r.get("rpm") or 0) < FAN["FAIL_RPM"]]
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Mode", "☀️ Day" if day else "🌙 Night")
        m2.metric("Compressor", "ON (solar)" if st.session_state.get("comp") else "OFF")
        m3.metric("Active alerts", len(alerts))
        m4.metric("Fan faults", len(faults))
        avg_t = sum(r["temp"] for r in latest.values()) / len(latest)
        st.metric("Average room temp", f"{avg_t:.1f} °C",
                  delta=f"{avg_t - FAN['SP']:+.1f} vs {FAN['SP']:.0f} °C target",
                  delta_color="inverse")
        if alerts:
            st.error(f"SMS dispatched to farmer · Zone(s) {', '.join(alerts)}")
        if faults:
            st.warning(f"Fan fault · Zone(s) {', '.join(faults)} (commanded but no rotation)")

    # --- fan energy KPIs -----------------------------------------------------
    e = st.session_state.energy
    base_w = len(ZONES) * FAN["RATED_W"]
    base_wh = e.get("baseline_wh", 0.0)
    saved = 100 * (1 - e.get("energy_wh", 0.0) / base_wh) if base_wh > 0 else 0.0
    st.subheader("Fan energy optimisation")
    k1, k2, k3, k4 = st.columns(4)
    k1.metric("Fan power now", f"{e.get('power_w', 0.0):.2f} W", f"vs {base_w:.1f} W fixed 100 %",
              delta_color="off")
    k2.metric("Energy used", f"{e.get('energy_wh', 0.0):.3f} Wh")
    k3.metric("Fixed-100 % baseline", f"{base_wh:.3f} Wh")
    k4.metric("Energy saved", f"{saved:.0f} %", f"{base_wh - e.get('energy_wh', 0.0):.3f} Wh less heat load",
              delta_color="normal")

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
                        width="stretch")
        a, b = st.columns(2)
        a.plotly_chart(line_chart(df, "temp", "Temperature (°C)", " °C", TEMP_LIMIT_C),
                       width="stretch")
        b.plotly_chart(line_chart(df, "hum", "Relative humidity (%)", " %"),
                       width="stretch")
        if "fan" in df and df["fan"].notna().any():
            st.plotly_chart(line_chart(df, "fan", "Fan speed command per zone (%)", " %",
                                       yrange=[0, 105]), width="stretch")


live_panel()
