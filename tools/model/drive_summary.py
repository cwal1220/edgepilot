"""주행 기록(route) 요약. 주행 뒤 손으로 하던 분석을 한 번에 낸다(2026-10 주행 검토들).

route마다 낸다:
  - 길이, 달린 시간(10 km/h 넘게), 최고·평균 속도
  - 달린 시간 중 결합·조향(active) 비율, 결합 구간, 해제와 그 사유, 결합 거부와 그 사유
  - 결합했는데 조향하지 않은 구간의 사유(달리는 중)
  - 모델 경로 대체(Laneless) 비율과 분당 진입(조향 중, 20 km/h 넘게)
  - 운전자 핸들 조작(5틱 히스테리시스 비트): 2초 안에 이어진 것은 한 번. 깜빡이·차선 변경 앞뒤 2초 밖에서 조향
    명령과 반대로 민 것을 시간당으로(대체 중이었는지도)
  - 85° 정지, 해제 예고, 조향 한계, 차선 변경, 조향 고장 시간
  - 모델: 프레임률, 빠진 프레임, 실행 시간, 촬영 → 발행 지연
  - 보정: 광류를 쓴 비율, 워프 yaw·pitch(처음 → 끝), 광류 장착 변경
  - 끝의 학습값(조향비, 횡가속 배율, 마찰, 각도 오프셋, lagd 지연)
  - Panda: 차단된 송신, 하트비트 손실, 수신 오류(주행 중 늘어난 수), 결함, 전압

사용:
    python tools/model/drive_summary.py <route_dir> [<route_dir> ...] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import recording_reader as rr

# ControlState::hud_flags 비트(src/common/ipc_messages.h kHudFlag*)
HUD_LANELESS = 1 << 0
HUD_SOFT_DISABLING = 1 << 2
HUD_STEER_SATURATED = 1 << 3
HUD_LANE_CHANGE_PENDING = 1 << 4
HUD_LANE_CHANGING = 1 << 5
HUD_STEER_PAUSED = 1 << 7
HUD_STEER_PAUSED_BY_DRIVER = 1 << 8
HUD_STEERING_PRESSED = 1 << 13

MOVING_KPH = 10.0
STEERING_KPH = 20.0
GUARD_S = 2.0          # 깜빡이·차선 변경 앞뒤로 운전자 조작에서 빼는 시간
PRESS_MERGE_S = 2.0    # 이보다 가깝게 이어진 조작은 한 번으로 센다
COMMAND_LEAD_S = 0.2   # 조작 직전 이만큼의 조향 명령과 방향을 견준다


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """참인 구간들의 [시작, 끝) 인덱스."""
    edges = np.diff(np.r_[0, mask.astype(np.int8), 0])
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def near(mask: np.ndarray, half: int) -> np.ndarray:
    """mask가 참인 표본에서 앞뒤 half 표본 안."""
    if half <= 0 or not mask.any():
        return mask.copy()
    return np.convolve(mask.astype(np.float32), np.ones(2 * half + 1, np.float32), "same") > 0


def text(value) -> str:
    return value.decode(errors="replace") if isinstance(value, bytes) else str(value)


def read_route(route: Path) -> dict[str, dict[str, np.ndarray]]:
    """요약에 쓰는 필드만 청크마다 복사해 이어 붙인다."""
    control_keys = ("timestamp_ns", "engaged", "active", "steering_fault", "left_blinker", "right_blinker",
                    "apply_torque", "driver_torque", "hud_flags", "ego_speed_kph", "active_block",
                    "engage_reject_event_id", "engage_reject_block")
    learner_keys = ("timestamp_ns", "steer_ratio", "lat_accel_factor", "friction", "angle_offset_average_deg")
    panda_keys = ("timestamp_ns", "blocked_msg_cnt", "heartbeat_lost", "can_rx_errs", "faults", "voltage")
    parts: dict[str, list[dict[str, np.ndarray]]] = {"control": [], "model": [], "learner": [], "loc": [],
                                                     "panda": []}
    for chunk in rr.iter_states(route, rr.RECORD_CONTROL_STATE, rr.RECORD_MODEL_STATE, rr.RECORD_LEARNER_STATE,
                                rr.RECORD_LOCALIZATION, rr.RECORD_PANDA_STATE):
        _, c = chunk[rr.RECORD_CONTROL_STATE]
        parts["control"].append({k: c[k].copy() for k in control_keys})
        log_t, m = chunk[rr.RECORD_MODEL_STATE]
        cal, flow = m["calibration"], m["flow_calibration"]
        parts["model"].append({
            "log_t": log_t.astype(np.int64), "capture_t": m["capture_timestamp_ns"].astype(np.int64),
            "frame_id": m["frame_id"].astype(np.int64), "exec_ms": m["model_execution_ms"].copy(),
            "yaw": cal["yaw"].copy(), "pitch": cal["pitch"].copy(),
            "source": m["calibration_source"]["source"].copy(), "mount_changes": flow["mount_changes"].copy()})
        _, learner = chunk[rr.RECORD_LEARNER_STATE]
        parts["learner"].append({k: learner[k].copy() for k in learner_keys})
        _, loc = chunk[rr.RECORD_LOCALIZATION]
        parts["loc"].append({"lateral_delay_s": loc["lateral_delay_s"].copy()})
        _, panda = chunk[rr.RECORD_PANDA_STATE]
        parts["panda"].append({k: panda[k].copy() for k in panda_keys})
    return {name: rr.concat_chunks(p) for name, p in parts.items() if p}


def press_episodes(c: dict[str, np.ndarray], t: np.ndarray, steering: np.ndarray, guard: np.ndarray,
                   laneless: np.ndarray, dt: float) -> list[dict]:
    """조향 중(20 km/h 넘게) 운전자가 핸들을 잡은 구간. 가까운 것은 하나로 묶는다."""
    pressed = (c["hud_flags"] & HUD_STEERING_PRESSED) != 0
    lead = max(1, int(round(COMMAND_LEAD_S / dt)))
    episodes: list[dict] = []
    for a, b in runs(pressed & steering):
        driver = float(np.mean(c["driver_torque"][a:b]))
        command = float(np.mean(c["apply_torque"][max(a - lead, 0):a + 1]))
        episode = {"t_s": float(t[a]), "duration_s": float((b - a) * dt),
                   "opposing": bool(abs(command) >= 20 and np.sign(driver) != np.sign(command)),
                   "blinker_or_lane_change": bool(guard[a:b].any()), "laneless": bool(laneless[a:b].any())}
        last = episodes[-1] if episodes else None
        if last and episode["t_s"] - (last["t_s"] + last["duration_s"]) < PRESS_MERGE_S:
            last["duration_s"] = episode["t_s"] + episode["duration_s"] - last["t_s"]
            last["opposing"] |= episode["opposing"]
            last["blinker_or_lane_change"] |= episode["blinker_or_lane_change"]
            last["laneless"] |= episode["laneless"]
        else:
            episodes.append(episode)
    return episodes


def summarize(route: Path) -> dict:
    data = read_route(route)
    c = data["control"]
    t = (c["timestamp_ns"] - c["timestamp_ns"][0]).astype(np.float64) * 1e-9
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    v = c["ego_speed_kph"].astype(np.float64)
    hud = c["hud_flags"]
    engaged, active = c["engaged"] == 1, c["active"] == 1
    moving = v > MOVING_KPH
    steering = active & (v > STEERING_KPH)
    laneless = (hud & HUD_LANELESS) != 0
    lane_change = (hud & (HUD_LANE_CHANGE_PENDING | HUD_LANE_CHANGING)) != 0
    blinker = (c["left_blinker"] | c["right_blinker"]) != 0
    guard = near(blinker | lane_change, int(round(GUARD_S / dt)))
    minutes = lambda mask: float(mask.sum() * dt / 60.0)  # noqa: E731
    share = lambda part, whole: float(part.sum() / whole.sum()) if whole.any() else 0.0  # noqa: E731

    out: dict = {"route": route.name, "minutes": float(t[-1] / 60.0), "moving_minutes": minutes(moving),
                 "max_kph": float(v.max()), "mean_moving_kph": float(v[moving].mean()) if moving.any() else 0.0,
                 "engaged_share_moving": share(engaged & moving, moving),
                 "active_share_moving": share(active & moving, moving)}
    out["engaged_spans"] = [{"t_s": float(t[a]), "minutes": float((b - a) * dt / 60.0)} for a, b in runs(engaged)]
    out["disengagements"] = []
    for i in np.flatnonzero(engaged[:-1] & ~engaged[1:]) + 1:
        reason = text(c["active_block"][i]) or text(c["active_block"][i - 1])
        out["disengagements"].append({"t_s": float(t[i]), "kph": float(v[i]), "reason": reason})
    reject = c["engage_reject_event_id"].astype(np.int64)
    out["engage_rejects"] = [{"t_s": float(t[i]), "reason": text(c["engage_reject_block"][i])}
                             for i in np.flatnonzero(np.diff(reject) != 0) + 1 if reject[i] != 0]
    blocks: dict[str, dict] = {}
    for a, b in runs(engaged & ~active & moving):
        entry = blocks.setdefault(text(c["active_block"][a]) or "?", {"count": 0, "seconds": 0.0})
        entry["count"] += 1
        entry["seconds"] += float((b - a) * dt)
    out["not_steering_while_engaged"] = blocks

    entries = runs(laneless & steering)
    out["laneless_share"] = share(laneless & steering, steering)
    out["laneless_entries_per_min"] = len(entries) / minutes(steering) if steering.any() else 0.0

    presses = press_episodes(c, t, steering, guard, laneless, dt)
    plain = [p for p in presses if p["opposing"] and not p["blinker_or_lane_change"]]
    hours = steering.sum() * dt / 3600.0
    out["presses"] = {"episodes": len(presses), "opposing_without_blinker": len(plain),
                      "opposing_per_hour": len(plain) / hours if hours > 0 else 0.0,
                      "opposing_in_laneless": sum(p["laneless"] for p in plain), "list": presses}

    paused = runs((hud & HUD_STEER_PAUSED) != 0)
    out["pauses_85"] = {"count": len(paused),
                        "by_driver": sum(bool(((hud[a:b] & HUD_STEER_PAUSED_BY_DRIVER) != 0).any()) for a, b in paused)}
    out["soft_disables"] = len(runs((hud & HUD_SOFT_DISABLING) != 0))
    out["steer_saturated"] = len(runs((hud & HUD_STEER_SATURATED) != 0))
    out["lane_changes"] = len(runs((hud & HUD_LANE_CHANGING) != 0))
    out["steering_fault_s"] = float(((c["steering_fault"] == 1) & engaged).sum() * dt)

    m = data.get("model")
    if m is not None and len(m["frame_id"]):
        span = (m["capture_t"][-1] - m["capture_t"][0]) * 1e-9
        gaps = np.diff(m["frame_id"])
        latency = (m["log_t"] - m["capture_t"]) * 1e-6
        deg = 180.0 / np.pi
        out["model"] = {"fps": float((len(m["frame_id"]) - 1) / span) if span > 0 else 0.0,
                        "lost_frames": int(np.sum(gaps[gaps > 1] - 1)),
                        "exec_ms_p50": float(np.percentile(m["exec_ms"], 50)),
                        "exec_ms_p99": float(np.percentile(m["exec_ms"], 99)),
                        "latency_ms_p50": float(np.percentile(latency, 50)),
                        "latency_ms_p99": float(np.percentile(latency, 99))}
        out["calibration"] = {"flow_share": float(np.mean(m["source"] == 1)),
                              "yaw_deg": [float(m["yaw"][0] * deg), float(m["yaw"][-1] * deg)],
                              "pitch_deg": [float(m["pitch"][0] * deg), float(m["pitch"][-1] * deg)],
                              "mount_changes": int(m["mount_changes"][-1]) - int(m["mount_changes"][0])}
    learner = data.get("learner")
    if learner is not None and len(learner["timestamp_ns"]):
        out["learners_end"] = {k: float(learner[k][-1]) for k in ("steer_ratio", "lat_accel_factor", "friction",
                                                                   "angle_offset_average_deg")}
        loc = data.get("loc")
        if loc is not None and len(loc["lateral_delay_s"]):
            out["learners_end"]["lateral_delay_s"] = float(loc["lateral_delay_s"][-1])
    panda = data.get("panda")
    if panda is not None and len(panda["timestamp_ns"]):
        grown = lambda k: int(panda[k][-1].astype(np.int64) - panda[k][0].astype(np.int64))  # noqa: E731
        out["panda"] = {"blocked_msgs": grown("blocked_msg_cnt"), "heartbeat_lost": grown("heartbeat_lost"),
                        "can_rx_errors": grown("can_rx_errs"), "faults_max": int(panda["faults"].max()),
                        "voltage_v": [float(panda["voltage"].min() / 1000.0), float(panda["voltage"].max() / 1000.0)]}
    return out


def print_summary(s: dict) -> None:
    print(f"== {s['route']}: {s['minutes']:.1f} min, moving {s['moving_minutes']:.1f} min, "
          f"max {s['max_kph']:.0f} km/h, mean {s['mean_moving_kph']:.0f} km/h")
    print(f"  engaged {s['engaged_share_moving'] * 100:.0f}% of moving, steering {s['active_share_moving'] * 100:.0f}%; "
          f"{len(s['engaged_spans'])} engaged span(s)")
    for d in s["disengagements"]:
        print(f"  disengaged at {d['t_s']:.1f} s, {d['kph']:.0f} km/h: {d['reason'] or '(none)'}")
    for r in s["engage_rejects"]:
        print(f"  engage refused at {r['t_s']:.1f} s: {r['reason']}")
    if s["not_steering_while_engaged"]:
        listed = ", ".join(f"{k} {v['count']}x {v['seconds']:.0f}s" for k, v in s["not_steering_while_engaged"].items())
        print(f"  engaged but not steering while moving: {listed}")
    print(f"  laneless {s['laneless_share'] * 100:.1f}% of steering, {s['laneless_entries_per_min']:.2f}/min")
    p = s["presses"]
    print(f"  driver presses {p['episodes']}; opposing without blinker/lane change {p['opposing_without_blinker']} "
          f"({p['opposing_per_hour']:.1f}/h, {p['opposing_in_laneless']} in laneless)")
    print(f"  85° pauses {s['pauses_85']['count']} ({s['pauses_85']['by_driver']} handed to the driver), "
          f"soft disables {s['soft_disables']}, steer limit {s['steer_saturated']}, lane changes {s['lane_changes']}, "
          f"steering fault {s['steering_fault_s']:.1f} s")
    if "model" in s:
        m = s["model"]
        print(f"  model {m['fps']:.2f} fps, lost {m['lost_frames']}, exec p50/p99 {m['exec_ms_p50']:.1f}/"
              f"{m['exec_ms_p99']:.1f} ms, capture→publish p50/p99 {m['latency_ms_p50']:.1f}/{m['latency_ms_p99']:.1f} ms")
        cal = s["calibration"]
        print(f"  calibration flow {cal['flow_share'] * 100:.0f}%, yaw {cal['yaw_deg'][0]:+.2f}→{cal['yaw_deg'][1]:+.2f}°, "
              f"pitch {cal['pitch_deg'][0]:+.2f}→{cal['pitch_deg'][1]:+.2f}°, mount changes {cal['mount_changes']}")
    if "learners_end" in s:
        le = s["learners_end"]
        print("  learners at the end: " + ", ".join(f"{k} {v:.4g}" for k, v in le.items()))
    if "panda" in s:
        pd = s["panda"]
        print(f"  panda blocked {pd['blocked_msgs']}, heartbeat lost {pd['heartbeat_lost']}, rx errors "
              f"{pd['can_rx_errors']}, faults {pd['faults_max']}, {pd['voltage_v'][0]:.1f}–{pd['voltage_v'][1]:.1f} V")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path)
    parser.add_argument("--json", type=Path, help="write the summaries as JSON")
    args = parser.parse_args()
    summaries = []
    for route in args.routes:
        if not rr.route_event_files(route):
            print(f"{route}: no events/*.bin", file=sys.stderr)
            sys.exit(1)
        summary = summarize(route)
        print_summary(summary)
        summaries.append(summary)
    if args.json:
        args.json.write_text(json.dumps(summaries, indent=1, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
