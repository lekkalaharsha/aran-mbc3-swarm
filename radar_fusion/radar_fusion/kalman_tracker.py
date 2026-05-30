"""
Kalman track manager — MBC-3 radar fusion.

State per track : [px, py, pz, vx, vy, vz]  (world frame, m / m·s⁻¹)
Observation     : [px, py, pz]

Cycle per update:
  1. Spatial merge of raw obs from all drones (multi-drone same-target collapse)
  2. Predict all existing tracks to current time
  3. Greedy NN association (gate_m)
  4. Kalman update matched; init new tracks for unmatched
  5. Prune tracks exceeding TTL
"""

import math
import time

import numpy as np


class KalmanTrack:

    def __init__(self, track_id: str, pos: np.ndarray, vel: np.ndarray,
                 q_pos: float, q_vel: float, r_pos: float,
                 now: float | None = None):
        self.id = track_id

        self.x = np.array([pos[0], pos[1], pos[2],
                            vel[0], vel[1], vel[2]], dtype=float)

        self.P = np.diag([r_pos, r_pos, r_pos, 10.0, 10.0, 10.0])

        self._q_pos = q_pos
        self._q_vel = q_vel
        self._R = np.diag([r_pos, r_pos, r_pos])

        self._H = np.zeros((3, 6))
        self._H[0, 0] = 1.0
        self._H[1, 1] = 1.0
        self._H[2, 2] = 1.0

        t0 = now if now is not None else time.time()
        self._last_predict_t: float = t0
        self.last_updated: float = t0
        self.sources: list[str] = []
        self.n_obs: int = 1

    def predict(self, now: float) -> None:
        dt = now - self._last_predict_t
        if dt <= 0.0:
            return
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        # Q scaled by dt so noise accumulation is proportional to time step.
        Q = np.diag([self._q_pos * dt] * 3 + [self._q_vel * dt] * 3)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._last_predict_t = now

    def update(self, z: np.ndarray, sources: list[str],
               now: float | None = None) -> None:
        y = z - self._H @ self.x
        S = self._H @ self.P @ self._H.T + self._R
        # solve avoids explicit inv(S) — numerically stable when S is near-singular.
        K = np.linalg.solve(S.T, (self.P @ self._H.T).T).T
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self._H) @ self.P
        self.last_updated = now if now is not None else time.time()
        self.sources = list(set(self.sources + sources))
        self.n_obs += 1

    def as_dict(self) -> dict:
        px, py, pz, vx, vy, vz = (float(v) for v in self.x)
        rng = math.sqrt(px * px + py * py + pz * pz)
        az  = math.degrees(math.atan2(py, px))
        el  = math.degrees(math.asin(pz / rng)) if rng > 0.0 else 0.0
        return {
            'id':       self.id,
            'pos':      [round(px, 2), round(py, 2), round(pz, 2)],
            'vel':      [round(vx, 2), round(vy, 2), round(vz, 2)],
            'range_m':  round(rng, 2),
            'az_deg':   round(az, 1),
            'el_deg':   round(el, 1),
            'sources':  self.sources,
            'n_obs':    self.n_obs,
            'timestamp': self.last_updated,
        }


class TrackManager:

    def __init__(self,
                 merge_dist_m: float = 5.0,
                 gate_m: float = 10.0,
                 ttl_s: float = 3.0,
                 q_pos: float = 1.0,
                 q_vel: float = 0.5,
                 r_pos: float = 5.0):
        self.merge_dist_m = merge_dist_m
        self.gate_m       = gate_m
        self.ttl_s        = ttl_s
        self._q_pos       = q_pos
        self._q_vel       = q_vel
        self._r_pos       = r_pos
        self._tracks: list[KalmanTrack] = []
        self._next_id: int = 1

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _new_id(self) -> str:
        tid = f'FUSED_{self._next_id:03d}'
        self._next_id += 1
        return tid

    def _spatial_merge(self, obs: list[dict]) -> list[dict]:
        """Collapse obs within merge_dist_m — handles multi-drone same target."""
        used   = [False] * len(obs)
        merged = []
        for i, oi in enumerate(obs):
            if used[i]:
                continue
            group   = [oi]
            used[i] = True
            pi      = np.array(oi['pos'], dtype=float)
            for j in range(i + 1, len(obs)):
                if used[j]:
                    continue
                pj = np.array(obs[j]['pos'], dtype=float)
                if float(np.linalg.norm(pi - pj)) <= self.merge_dist_m:
                    group.append(obs[j])
                    used[j] = True
            pos = np.mean([np.array(g['pos'], dtype=float) for g in group], axis=0)
            vel = np.mean([np.array(g['vel'], dtype=float) for g in group], axis=0)
            merged.append({
                'pos':     pos,
                'vel':     vel,
                'sources': [g['source'] for g in group],
            })
        return merged

    # ── Public API ───────────────────────────────────────────────────────────

    def update(self, raw: dict[str, list], now: float) -> None:
        """raw: {drone_id: [target_dict, ...]} — all targets from all drones."""
        obs: list[dict] = []
        for drone, targets in raw.items():
            for t in targets:
                obs.append({
                    'pos':    t['pos'],
                    'vel':    t.get('vel', [0.0, 0.0, 0.0]),
                    'source': drone,
                })

        measurements = self._spatial_merge(obs) if obs else []

        # Predict all tracks forward to now
        for track in self._tracks:
            track.predict(now)

        # Greedy NN association
        matched_tracks: set[int] = set()
        matched_meas:   set[int] = set()

        pairs: list[tuple] = []
        for ti, track in enumerate(self._tracks):
            tp = track.x[:3]
            for mi, m in enumerate(measurements):
                d = float(np.linalg.norm(tp - m['pos']))
                if d <= self.gate_m:
                    pairs.append((d, ti, mi))

        pairs.sort(key=lambda a: a[0])

        for _, ti, mi in pairs:
            if ti in matched_tracks or mi in matched_meas:
                continue
            m = measurements[mi]
            self._tracks[ti].update(m['pos'], m['sources'], now)
            matched_tracks.add(ti)
            matched_meas.add(mi)

        # New tracks for unmatched measurements
        for mi, m in enumerate(measurements):
            if mi not in matched_meas:
                t = KalmanTrack(
                    self._new_id(),
                    m['pos'], m['vel'],
                    self._q_pos, self._q_vel, self._r_pos,
                    now=now,
                )
                t.sources = m['sources']
                self._tracks.append(t)

        # Prune stale
        self._tracks = [
            t for t in self._tracks
            if (now - t.last_updated) <= self.ttl_s
        ]

    def get_fused_tracks(self) -> list[dict]:
        return [t.as_dict() for t in self._tracks]

    def get_dropped_ids(self, live_ids: set[str]) -> list[str]:
        """Return IDs from previous cycle no longer in live tracks (for marker cleanup)."""
        current = {t.id for t in self._tracks}
        return list(live_ids - current)
