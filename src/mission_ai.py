#!/usr/bin/env python3
"""
mission_ai.py — Ollama LLM operator intent interface for MBC-3 swarm.

Translates natural language operator commands into structured mission deltas.
Uses local Ollama (http://localhost:11434) — no API key, no cost.
Falls back to rule-based parser if Ollama is not running.

Integration:
    from mission_ai import MissionAI
    ai = MissionAI()                          # auto-detects Ollama
    delta = ai.parse_command("Redirect DRONE-2 to orbit ALPHA-2 for 30 seconds",
                             swarm_state=drone_states)
    ok, reason = ai.validate(delta)
    if ok:
        apply_delta(delta)

Ollama setup (one-time):
    curl -fsSL https://ollama.com/install.sh | sh
    ollama pull phi3:mini          # 2.3 GB — fast, good JSON
    ollama serve                   # starts on port 11434
"""

import json
import os
import re
import sys
from typing import Optional

import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_config import SECONDARY_TARGETS, NO_FLY_ZONES, ALTITUDE, SPEED
from mission_config_swarm import BASE_ALT as CRUISE_ALT, ALT_SEP

OLLAMA_URL    = os.environ.get("OLLAMA_URL", "http://localhost:11434")
MODEL         = os.environ.get("MISSION_AI_MODEL", "phi3:mini")
API_TIMEOUT_S = 30.0   # local inference can be slow first call

VALID_ACTIONS = {
    "reassign_sector",  # move drone to a different survey sector
    "change_altitude",  # adjust cruise altitude for one or more drones
    "change_speed",     # adjust mission speed (m/s)
    "orbit_target",     # loiter around a named or lat/lon target
    "rtl_drone",        # return-to-launch a specific drone
    "abort_mission",    # emergency stop (requires reason)
    "loiter",           # hold position at current location
    "resume_mission",   # resume after hold
    "no_action",        # command ambiguous or no change needed
}

_NFZ_NAMES = ", ".join(n["name"] for n in NO_FLY_ZONES)

_SYSTEM_PROMPT = (
    "You are the mission controller AI for an autonomous 5-drone FMCW radar swarm "
    "(MBC-3 program, IAF competition).\n\n"
    "Your job: translate the operator's natural language command into a structured mission delta JSON.\n\n"
    "SWARM CONTEXT:\n"
    f"- 5 drones: DRONE-0 through DRONE-4. Higher index = higher election priority.\n"
    f"- Cruise altitudes: DRONE-i flies at {int(CRUISE_ALT)}m + i×{int(ALT_SEP)}m AGL.\n"
    "- Survey grid partitioned round-robin by row index.\n"
    "- Leader drone (highest-index alive) controls radar and reassignment.\n"
    f"- No-fly zones: {_NFZ_NAMES}\n\n"
    "SECONDARY TARGETS (for orbit_target action):\n"
    + "\n".join(
        f"  {t['name']}: lat={t['lat']} lon={t['lon']} r={t['orbit_radius_m']}m"
        for t in SECONDARY_TARGETS
    )
    + "\n\n"
    "VALID ACTIONS:\n"
    "  reassign_sector   — move drone to different grid sector (params: sector_idx int 0-3)\n"
    "  change_altitude   — adjust AGL altitude (params: altitude_m float)\n"
    "  change_speed      — adjust speed (params: speed_ms float)\n"
    "  orbit_target      — loiter around target (params: target_name, lat, lon,\n"
    "                      orbit_radius_m, orbit_speed_ms, orbit_duration_s)\n"
    "  rtl_drone         — return drone home (params: {})\n"
    "  abort_mission     — emergency stop all (params: reason string — required)\n"
    "  loiter            — hold position (params: {})\n"
    "  resume_mission    — resume after hold (params: {})\n"
    "  no_action         — ambiguous or nothing to do (params: raw string)\n\n"
    'OUTPUT FORMAT — output ONLY valid JSON, no prose, no markdown fences:\n'
    '{"action":"<action_type>","target_drones":[<int>,...],'
    '"params":{<key-value pairs>},"confidence":<0.0-1.0>,"reasoning":"<one sentence>"}\n\n'
    "SAFETY RULES:\n"
    "- altitude_m must be in [50, 600].\n"
    "- speed_ms must be in [5, 40].\n"
    "- abort_mission requires a non-empty reason.\n"
    "- If confidence < 0.5 emit no_action instead.\n"
)


class MissionAI:
    """Ollama-backed operator command interpreter for MBC-3 swarm."""

    def __init__(self):
        self._available = _check_ollama()
        if self._available:
            print(f"[AI] Ollama OK — model={MODEL}", flush=True)
        else:
            print(f"[AI] Ollama not reachable at {OLLAMA_URL} — rule fallback active", flush=True)

    @property
    def llm_available(self) -> bool:
        return self._available

    def parse_command(
        self,
        command: str,
        swarm_state: Optional[dict] = None,
        failed_drones: Optional[list] = None,
        prior_reassignments: Optional[list] = None,
    ) -> dict:
        """
        Parse operator natural language → structured mission delta.
        Falls back to rule-based parser if Ollama unavailable or errors.
        """
        ctx = _build_context(swarm_state, failed_drones, prior_reassignments)

        if self._available:
            try:
                result = _ollama_parse(command, ctx)
                result["source"] = "llm"
                return result
            except Exception as exc:
                print(f"[AI] Ollama error ({exc}) — rule fallback", flush=True)

        result = _rule_parse(command, swarm_state or {})
        result["source"] = "rules"
        return result

    def validate(self, delta: dict, swarm_state: Optional[dict] = None) -> tuple[bool, str]:
        """
        Validate mission delta before execution.
        Returns (ok, reason). reason is empty string when ok=True.
        """
        action = delta.get("action")
        if action not in VALID_ACTIONS:
            return False, f"unknown action '{action}'"

        targets = delta.get("target_drones", [])
        if not isinstance(targets, list):
            return False, "target_drones must be a list"
        for t in targets:
            if not isinstance(t, int) or not (0 <= t <= 4):
                return False, f"invalid drone index {t}"

        params = delta.get("params", {})

        if action == "change_altitude":
            alt = params.get("altitude_m")
            if alt is None:
                return False, "change_altitude requires params.altitude_m"
            if not (50 <= float(alt) <= 600):
                return False, f"altitude {alt}m outside safe range [50, 600]"

        if action == "change_speed":
            spd = params.get("speed_ms")
            if spd is None:
                return False, "change_speed requires params.speed_ms"
            if not (5 <= float(spd) <= 40):
                return False, f"speed {spd}m/s outside range [5, 40]"

        if action == "abort_mission":
            if not params.get("reason"):
                return False, "abort_mission requires params.reason"

        conf = float(delta.get("confidence", 1.0))
        if conf < 0.5:
            return False, f"confidence {conf:.2f} below threshold 0.50"

        return True, ""


# ── Ollama helpers ─────────────────────────────────────────────────────────────

def _check_ollama() -> bool:
    try:
        r = requests.get(f"{OLLAMA_URL}/api/tags", timeout=2.0)
        return r.status_code == 200
    except Exception:
        return False


def _ollama_parse(command: str, context: str) -> dict:
    user_msg = f"CURRENT SWARM STATE:\n{context}\n\nOPERATOR COMMAND:\n{command}"

    payload = {
        "model":  MODEL,
        "stream": False,
        "messages": [
            {"role": "system", "content": _SYSTEM_PROMPT},
            {"role": "user",   "content": user_msg},
        ],
        "options": {"temperature": 0.1},   # low temp for deterministic JSON
    }

    r = requests.post(
        f"{OLLAMA_URL}/api/chat",
        json=payload,
        timeout=API_TIMEOUT_S,
    )
    r.raise_for_status()

    text = r.json()["message"]["content"].strip()

    # Strip markdown fences if model wraps output
    if text.startswith("```"):
        parts = text.split("```")
        text = parts[1][4:].strip() if parts[1].startswith("json") else parts[1].strip()

    # Extract first JSON object if model adds prose before/after
    m = re.search(r'\{.*\}', text, re.DOTALL)
    if m:
        text = m.group(0)

    return json.loads(text)


# ── Context builder ────────────────────────────────────────────────────────────

def _build_context(
    swarm_state: Optional[dict],
    failed_drones: Optional[list],
    prior_reassignments: Optional[list],
) -> str:
    lines = []

    if swarm_state:
        lines.append("Drones:")
        for idx in range(5):
            s = swarm_state.get(idx) or swarm_state.get(str(idx)) or {}
            lines.append(
                f"  DRONE-{idx}: alt={s.get('alt','?')}m "
                f"phase={s.get('phase','?')} armed={s.get('armed','?')}"
            )

    if failed_drones:
        lines.append(f"Failed: DRONE-{', DRONE-'.join(str(d) for d in failed_drones)}")

    if prior_reassignments:
        lines.append("Prior reassignments (last 3):")
        for r in prior_reassignments[-3:]:
            lines.append(f"  {r}")

    return "\n".join(lines) if lines else "No swarm state available."


# ── Rule-based fallback ────────────────────────────────────────────────────────

def _rule_parse(command: str, swarm_state: dict) -> dict:
    """Keyword-based parser — no API required."""
    cmd = command.lower()

    if any(k in cmd for k in ("rtl", "return", "go home", "land")):
        return {
            "action":        "rtl_drone",
            "target_drones": _extract_drones(cmd),
            "params":        {},
            "confidence":    0.80,
            "reasoning":     "RTL keyword matched",
        }

    if any(k in cmd for k in ("abort", "emergency stop", "stop all", "cancel mission")):
        return {
            "action":        "abort_mission",
            "target_drones": [],
            "params":        {"reason": command},
            "confidence":    0.90,
            "reasoning":     "Abort keyword matched",
        }

    if any(k in cmd for k in ("altitude", "climb", "descend", "height", "agl")):
        alt = _extract_number(cmd, lo=50, hi=600)
        if alt is not None:
            return {
                "action":        "change_altitude",
                "target_drones": _extract_drones(cmd),
                "params":        {"altitude_m": alt},
                "confidence":    0.75,
                "reasoning":     "Altitude keyword + number matched",
            }

    if any(k in cmd for k in ("speed", "faster", "slower", "m/s")):
        spd = _extract_number(cmd, lo=5, hi=40)
        if spd is not None:
            return {
                "action":        "change_speed",
                "target_drones": _extract_drones(cmd),
                "params":        {"speed_ms": spd},
                "confidence":    0.75,
                "reasoning":     "Speed keyword + number matched",
            }

    for tgt in SECONDARY_TARGETS:
        short = tgt["name"].split("-")[0].lower()
        full  = tgt["name"].lower()
        if short in cmd or full in cmd or tgt["name"].split()[0].lower() in cmd:
            return {
                "action":        "orbit_target",
                "target_drones": _extract_drones(cmd),
                "params": {
                    "target_name":      tgt["name"],
                    "lat":              tgt["lat"],
                    "lon":              tgt["lon"],
                    "orbit_radius_m":   tgt["orbit_radius_m"],
                    "orbit_speed_ms":   tgt["orbit_speed_ms"],
                    "orbit_duration_s": tgt.get("orbit_duration_s", 20),
                },
                "confidence":    0.80,
                "reasoning":     f"Target '{tgt['name']}' matched in command",
            }

    if any(k in cmd for k in ("hold", "loiter", "hover", "wait", "pause")):
        return {
            "action":        "loiter",
            "target_drones": _extract_drones(cmd),
            "params":        {},
            "confidence":    0.70,
            "reasoning":     "Hold keyword matched",
        }

    if any(k in cmd for k in ("resume", "continue", "go")):
        return {
            "action":        "resume_mission",
            "target_drones": _extract_drones(cmd),
            "params":        {},
            "confidence":    0.65,
            "reasoning":     "Resume keyword matched",
        }

    return {
        "action":        "no_action",
        "target_drones": [],
        "params":        {"raw": command},
        "confidence":    0.30,
        "reasoning":     "No rule matched — command unclear",
    }


def _extract_drones(text: str) -> list:
    return [int(h) for h in re.findall(r'drone[-\s]?(\d)', text, re.IGNORECASE)]


def _extract_number(text: str, lo: float = 0.0, hi: float = 1e9) -> Optional[float]:
    for m in re.finditer(r'\b(\d+(?:\.\d+)?)\b', text):
        v = float(m.group(1))
        if lo <= v <= hi:
            return v
    return None


# ── CLI smoke-test ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ai = MissionAI()

    mock_state = {
        i: {"alt": 100 + i * 10, "phase": "SURVEY", "armed": True}
        for i in range(5)
    }

    tests = [
        "Return DRONE-2 to base immediately",
        "Climb DRONE-0 to 550 meters altitude",
        "Send DRONE-3 to orbit ALPHA-2 for recon",
        "Emergency stop — comms lost with all units",
        "Set speed to 15 m/s for DRONE-4",
        "Hold position all drones",
        "Resume mission for DRONE-0",
        "xyzzy frobnicate the swarm",
    ]

    print(f"\n{'CMD':<45} {'SRC':<6} {'ACTION':<20} {'OK'}", flush=True)
    print("-" * 90, flush=True)
    for cmd in tests:
        delta = ai.parse_command(cmd, swarm_state=mock_state)
        ok, reason = ai.validate(delta, mock_state)
        src    = delta.get("source", "?")
        status = "OK" if ok else f"FAIL:{reason[:30]}"
        print(
            f"{cmd:<45} {src:<6} {delta['action']:<20} {status}  "
            f"drones={delta['target_drones']}",
            flush=True,
        )
