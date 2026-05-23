#!/usr/bin/env python3
"""
mission_ai.py — Claude Haiku 4.5 operator intent interface for MBC-3 swarm.

Translates natural language operator commands into structured mission deltas.
Includes validation layer + rule-based greedy fallback when API is unavailable.

Integration:
    from mission_ai import MissionAI
    ai = MissionAI()
    delta = ai.parse_command("Redirect DRONE-2 to orbit ALPHA-2 for 30 seconds",
                             swarm_state=drone_states)
    ok, reason = ai.validate(delta)
    if ok:
        apply_delta(delta)
"""

import json
import os
import re
import sys
import time
from typing import Optional

try:
    import anthropic
    _ANTHROPIC_AVAILABLE = True
except ImportError:
    _ANTHROPIC_AVAILABLE = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_config import (
    SECONDARY_TARGETS, NO_FLY_ZONES,
    ALTITUDE, SPEED,
)

CRUISE_ALT = 100.0   # swarm_mission.py base altitude (m AGL)
ALT_SEP    = 10.0    # per-drone stagger (m)

MODEL         = "claude-haiku-4-5"
MAX_TOKENS    = 512
API_TIMEOUT_S = 8.0

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

_SYSTEM_PROMPT = f"""You are the mission controller AI for an autonomous 5-drone FMCW radar swarm (MBC-3 program, IAF competition).

Your job: translate the operator's natural language command into a structured mission delta JSON.

SWARM CONTEXT:
- 5 drones: DRONE-0 through DRONE-4. Higher index = higher election priority.
- Cruise altitudes: DRONE-i flies at {int(CRUISE_ALT)}m + i×{int(ALT_SEP)}m AGL.
- Survey grid partitioned round-robin by row index.
- Leader drone (highest-index alive) controls radar and reassignment.
- No-fly zones: {_NFZ_NAMES}

SECONDARY TARGETS (for orbit_target action):
""" + "\n".join(
    f"  {t['name']}: lat={t['lat']} lon={t['lon']} r={t['orbit_radius_m']}m"
    for t in SECONDARY_TARGETS
) + """

VALID ACTIONS:
  reassign_sector   — move drone to different grid sector (params: sector_idx int 0-3)
  change_altitude   — adjust AGL altitude (params: altitude_m float)
  change_speed      — adjust speed (params: speed_ms float)
  orbit_target      — loiter around target (params: target_name, lat, lon,
                      orbit_radius_m, orbit_speed_ms, orbit_duration_s)
  rtl_drone         — return drone home (params: {})
  abort_mission     — emergency stop all (params: reason string — required)
  loiter            — hold position (params: {})
  resume_mission    — resume after hold (params: {})
  no_action         — ambiguous or nothing to do (params: raw string)

OUTPUT FORMAT — JSON only, no prose, no markdown fences:
{
  "action": "<action_type>",
  "target_drones": [<int>, ...],
  "params": {<action-specific key-value pairs>},
  "confidence": <0.0-1.0>,
  "reasoning": "<one sentence>"
}

SAFETY RULES (enforce strictly in output):
- altitude_m must be in [50, 600] for MBC3_MODE.
- speed_ms must be in [5, 40].
- Never route drones into no-fly zones.
- abort_mission requires a non-empty reason.
- If confidence < 0.5 emit no_action instead.
"""


class MissionAI:
    """Claude Haiku 4.5 operator command interpreter for MBC-3 swarm."""

    def __init__(self, api_key: Optional[str] = None):
        self._client = None
        if _ANTHROPIC_AVAILABLE:
            key = api_key or os.environ.get("ANTHROPIC_API_KEY")
            if key:
                self._client = anthropic.Anthropic(api_key=key)

    @property
    def llm_available(self) -> bool:
        return self._client is not None

    def parse_command(
        self,
        command: str,
        swarm_state: Optional[dict] = None,
        failed_drones: Optional[list] = None,
        prior_reassignments: Optional[list] = None,
    ) -> dict:
        """
        Parse operator natural language → structured mission delta.
        Falls back to rule-based parser if LLM unavailable or errors.
        """
        ctx = _build_context(swarm_state, failed_drones, prior_reassignments)

        if self._client:
            try:
                result = self._llm_parse(command, ctx)
                result["source"] = "llm"
                return result
            except Exception as exc:
                print(f"[AI] LLM error ({exc}) — rule fallback", flush=True)

        result = _rule_parse(command, swarm_state or {})
        result["source"] = "rules"
        return result

    def _llm_parse(self, command: str, context: str) -> dict:
        user_msg = f"CURRENT SWARM STATE:\n{context}\n\nOPERATOR COMMAND:\n{command}"

        msg = self._client.messages.create(
            model=MODEL,
            max_tokens=MAX_TOKENS,
            system=_SYSTEM_PROMPT,
            messages=[{"role": "user", "content": user_msg}],
            timeout=API_TIMEOUT_S,
        )

        text = msg.content[0].text.strip()
        # Strip markdown fences if model wraps output
        if text.startswith("```"):
            parts = text.split("```")
            text = parts[1][4:] if parts[1].startswith("json") else parts[1]
        return json.loads(text)

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
        # Match "ALPHA" from "ALPHA-2 Industrial Compound"
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
    """Return first number in text within [lo, hi], skipping out-of-range values."""
    for m in re.finditer(r'\b(\d+(?:\.\d+)?)\b', text):
        v = float(m.group(1))
        if lo <= v <= hi:
            return v
    return None


# ── CLI smoke-test ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ai = MissionAI()
    print(f"[AI] model={MODEL}  llm_available={ai.llm_available}", flush=True)

    mock_state = {
        i: {"alt": 100 + i * 10, "phase": "SURVEY", "armed": True}
        for i in range(5)
    }
    mock_failed = [1]

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
        delta = ai.parse_command(cmd, swarm_state=mock_state, failed_drones=mock_failed)
        ok, reason = ai.validate(delta, mock_state)
        src    = delta.get("source", "?")
        status = "OK" if ok else f"FAIL:{reason[:30]}"
        print(
            f"{cmd:<45} {src:<6} {delta['action']:<20} {status}  "
            f"drones={delta['target_drones']}",
            flush=True,
        )
