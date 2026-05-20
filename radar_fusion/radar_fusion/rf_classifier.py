"""
Random Forest classifier — Layer 2 target gate for MBC-3.

Features per cluster (derived from PointCloud2 clustering):
  range_m    : distance to centroid (m)
  hits       : point count — proxy for radar cross section
  range_std  : std-dev of member point ranges — target compactness
  spread_xy  : lateral std-dev (m) — cluster tightness in XY plane
  el_deg     : elevation angle (deg)

Trained on synthetic data mimicking FMCW radar returns:
  Real targets  : many hits (15-100), compact, low spread
  Clutter/noise : few hits (3-8), scattered, high spread

In simulation (gpu_lidar, no multipath): all sphere clusters will have
high hit counts and compact geometry → classified as targets (correct).
"""

import os

import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler


FEATURE_NAMES = ['range_m', 'hits', 'range_std', 'spread_xy', 'el_deg']

_N_TRAIN      = 2000   # samples per class
_N_ESTIMATORS = 50     # small — fast inference on Jetson
_MAX_DEPTH    = 8
_RANDOM_STATE = 42


class RFTargetClassifier:
    """Train once on synthetic data at init; classify clusters in real time."""

    def __init__(self, model_path: str | None = None):
        self._clf    = None
        self._scaler = None

        if model_path and os.path.exists(model_path):
            self._load(model_path)
        else:
            self._train_synthetic()
            if model_path:
                d = os.path.dirname(model_path)
                if d:
                    os.makedirs(d, exist_ok=True)
                self._save(model_path)

    # ── Training ──────────────────────────────────────────────────────────────

    def _train_synthetic(self) -> None:
        rng = np.random.default_rng(_RANDOM_STATE)
        N   = _N_TRAIN

        # Real aerial targets: many hits, compact cluster
        # Lower bound 9 avoids overlap with clutter boundary at 8.
        t = np.column_stack([
            rng.uniform(20, 2100, N),       # range_m — up to 2km+
            rng.integers(9, 101, N).astype(float),   # hits  — 9+ (no overlap)
            rng.uniform(0.2, 1.5, N),       # range_std  (tight)
            rng.uniform(0.3, 2.5, N),       # spread_xy  (tight)
            rng.uniform(-5,   25, N),       # el_deg
        ])

        # Clutter / ground returns / noise: few hits, scattered
        # Upper bound 8 (exclusive) = max 7 hits — clean separation from targets.
        c = np.column_stack([
            rng.uniform(20,  500, N),       # range_m
            rng.integers(3,    8, N).astype(float),  # hits  3–7 (no overlap)
            rng.uniform(1.5,  8.0, N),      # range_std  (loose)
            rng.uniform(2.5, 10.0, N),      # spread_xy  (loose)
            rng.uniform(-5,   25, N),       # el_deg
        ])

        X = np.vstack([t, c])
        y = np.array([1] * N + [0] * N)

        self._scaler = StandardScaler()
        Xs = self._scaler.fit_transform(X)

        self._clf = RandomForestClassifier(
            n_estimators=_N_ESTIMATORS,
            max_depth=_MAX_DEPTH,
            random_state=_RANDOM_STATE,
            n_jobs=2,
        )
        self._clf.fit(Xs, y)

    def _save(self, path: str) -> None:
        joblib.dump({'clf': self._clf, 'scaler': self._scaler}, path)

    def _load(self, path: str) -> None:
        d = joblib.load(path)
        self._clf    = d['clf']
        self._scaler = d['scaler']

    # ── Inference ─────────────────────────────────────────────────────────────

    def predict(self, clusters: list[dict]) -> list[int]:
        """
        clusters: list of dicts — each must have FEATURE_NAMES keys.
        Returns list of int labels: 1 = confirmed target, 0 = clutter.
        Falls back to pass-all (1) if model not ready.
        """
        if not clusters or self._clf is None:
            return [1] * len(clusters)

        X = np.array(
            [[c[f] for f in FEATURE_NAMES] for c in clusters],
            dtype=float,
        )
        Xs = self._scaler.transform(X)
        return self._clf.predict(Xs).tolist()

    def predict_proba(self, clusters: list[dict]) -> list[float]:
        """Return confidence score (probability of class=1) per cluster."""
        if not clusters or self._clf is None:
            return [1.0] * len(clusters)

        X = np.array(
            [[c[f] for f in FEATURE_NAMES] for c in clusters],
            dtype=float,
        )
        Xs = self._scaler.transform(X)
        return self._clf.predict_proba(Xs)[:, 1].tolist()
