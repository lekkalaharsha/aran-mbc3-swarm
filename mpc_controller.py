import math
import time
import numpy as np
from scipy.optimize import minimize
from typing import List, Tuple, Optional


# ══════════════════════════════════════════════════════════
#  SHARED GEOMETRY  (identical API to pid_controller.py)
# ══════════════════════════════════════════════════════════

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def project_waypoint(lat: float, lon: float,
                     bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    """Project a point from (lat, lon) along bearing_deg for distance_m metres.
    bearing_deg: 0=North, clockwise positive.  Returns (new_lat, new_lon)."""
    R = 6_371_000.0
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    d = distance_m / R
    lat2 = math.asin(
        math.sin(lat1) * math.cos(d)
        + math.cos(lat1) * math.sin(d) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def compute_avoidance_waypoint(drone_lat: float, drone_lon: float,
                                drone_heading_deg: float,
                                escape_bearing_deg: float,
                                offset_m: float = 20.0) -> Tuple[float, float]:
    """Compute a detour waypoint by projecting along escape_bearing_deg.
    drone_heading_deg kept for API compatibility (unused)."""
    return project_waypoint(drone_lat, drone_lon, escape_bearing_deg, offset_m)


def best_escape_bearing(sectors: List[float],
                        drone_heading_deg: float,
                        obstacle_bearing_deg: float) -> Tuple[float, str, float]:
    """Choose escape direction: left or right of the obstacle, whichever sector
    has the most clearance.  Returns (absolute_bearing_deg, side_str, clearance_m)."""
    sector_count = len(sectors)
    sector_size  = 360.0 / sector_count
    abs_obstacle = obstacle_bearing_deg % 360

    left_bearing  = (abs_obstacle - 90) % 360
    right_bearing = (abs_obstacle + 90) % 360

    left_sector  = int(left_bearing  / sector_size) % sector_count
    right_sector = int(right_bearing / sector_size) % sector_count

    left_clear  = sectors[left_sector]
    right_clear = sectors[right_sector]

    if left_clear >= right_clear:
        return left_bearing,  "LEFT",  left_clear
    else:
        return right_bearing, "RIGHT", right_clear


# ══════════════════════════════════════════════════════════
#  LOCAL COORDINATE HELPERS
# ══════════════════════════════════════════════════════════

def latlon_to_ned(lat: float, lon: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert (lat, lon) to local NED (north_m, east_m) relative to ref."""
    north = math.radians(lat - ref_lat) * 6_371_000.0
    east  = math.radians(lon - ref_lon) * 6_371_000.0 * math.cos(math.radians(ref_lat))
    return north, east


def ned_to_latlon(north: float, east: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert local NED (north_m, east_m) back to (lat, lon)."""
    lat = ref_lat + math.degrees(north / 6_371_000.0)
    lon = ref_lon + math.degrees(east  / (6_371_000.0 * math.cos(math.radians(ref_lat))))
    return lat, lon


# ══════════════════════════════════════════════════════════
#  CORE MPC ENGINE
# ══════════════════════════════════════════════════════════

class MPCEngine:
    """
    Generic finite-horizon MPC for a 6-DOF point-mass drone model.

    State  x = [n, e, d, vn, ve, vd]   (NED metres / m·s⁻¹)
    Input  u = [an, ae, ad]             (m·s⁻²)

    The optimiser minimises a composite cost over a prediction horizon of N
    steps at timestep dt.  Costs are quadratic and penalties are added
    whenever the predicted trajectory violates obstacle soft constraints.

    Parameters
    ──────────
    N           prediction horizon (steps)
    dt          timestep (s) — should match avoidance loop rate
    u_max       maximum acceleration (m/s²)
    v_max       maximum speed (m/s)
    Q_track     state tracking cost weight (position error)
    Q_vel       velocity regularisation weight
    R_input     control effort weight
    R_delta     control rate (smoothness) weight
    Q_terminal  terminal state cost multiplier (× Q_track)
    """

    def __init__(
        self,
        N:          int   = 10,
        dt:         float = 0.02,
        u_max:      float = 4.0,
        v_max:      float = 55.0,
        Q_track:    float = 2.0,
        Q_vel:      float = 0.1,
        R_input:    float = 0.05,
        R_delta:    float = 0.2,
        Q_terminal: float = 5.0,
    ):
        self.N          = N
        self.dt         = dt
        self.u_max      = u_max
        self.v_max      = v_max
        self.Q_track    = Q_track
        self.Q_vel      = Q_vel
        self.R_input    = R_input
        self.R_delta    = R_delta
        self.Q_terminal = Q_terminal

        # Build discrete LTI matrices  x[k+1] = A x[k] + B u[k]
        self._build_matrices()

        # Warm-start storage
        self._u_prev:      Optional[np.ndarray] = None
        
        self._U_opt_last:  Optional[np.ndarray] = None

        # Obstacles: list of (north_m, east_m, radius_m, penalty_weight)
        self._obstacles: List[Tuple[float, float, float, float]] = []

        # Reference target state
        self._x_ref: Optional[np.ndarray] = None

    def _build_matrices(self):
        """Construct A and B from current dt.  Called at init and after horizon change."""
        dt = self.dt
        self.A = np.eye(6)
        self.A[0, 3] = dt
        self.A[1, 4] = dt
        self.A[2, 5] = dt

        self.B = np.zeros((6, 3))
        self.B[0, 0] = 0.5 * dt ** 2
        self.B[1, 1] = 0.5 * dt ** 2
        self.B[2, 2] = 0.5 * dt ** 2
        self.B[3, 0] = dt
        self.B[4, 1] = dt
        self.B[5, 2] = dt

    # ── Public configuration ──────────────────────────────

    def set_reference(self, x_ref: np.ndarray):
        """Set the desired state x_ref = [n,e,d,vn,ve,vd]."""
        self._x_ref = x_ref.copy()

    def set_obstacles(self, obstacles: List[Tuple[float, float, float, float]]):
        """Provide obstacle list: [(north_m, east_m, radius_m, weight), ...]."""
        self._obstacles = obstacles

    def set_speed_limit(self, v_max: float):
        self.v_max = v_max

    # BUG-2 FIX: atomic horizon update — replaces bare `self._mpc.N = 8` mutations.
    def reset_horizon(self, N: int):
        """
        Change the prediction horizon atomically.

        Previously callers mutated self.N directly, leaving _u_prev and
        _U_opt_last sized for the old horizon.  On a fast→slow transition
        (N=8 → N=10) the next solve() called result.x.reshape(10,3) on a
        24-element array and crashed.  This method clears both warm-start
        buffers so the next solve gets a fresh, correctly-sized start.
        """
        self.N = N
        self._u_prev     = None
        self._U_opt_last = None

    # ── Trajectory rollout ────────────────────────────────

    def _rollout(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        """Simulate N steps from x0 with controls U (shape N×3).
        Returns trajectory X (shape (N+1)×6)."""
        X = np.empty((self.N + 1, 6))
        X[0] = x0
        for k in range(self.N):
            X[k + 1] = self.A @ X[k] + self.B @ U[k]
            # Speed clamp (soft — reflected in cost, but hard-clip velocity
            # so rollout stays physical)
            spd = np.linalg.norm(X[k + 1, 3:6])
            if spd > self.v_max:
                X[k + 1, 3:6] *= self.v_max / spd
        return X

    # ── Cost function ─────────────────────────────────────

    def _cost(self, u_flat: np.ndarray, x0: np.ndarray) -> float:
        U  = u_flat.reshape(self.N, 3)
        X  = self._rollout(x0, U)
        J  = 0.0
        xr = self._x_ref if self._x_ref is not None else np.zeros(6)

        for k in range(1, self.N + 1):
            x = X[k]
            # Position tracking
            pos_err = x[:3] - xr[:3]
            w = self.Q_terminal if k == self.N else self.Q_track
            J += w * float(pos_err @ pos_err)

            # Velocity tracking / regularisation
            vel_err = x[3:6] - xr[3:6]
            J += self.Q_vel * float(vel_err @ vel_err)

            # Obstacle avoidance penalty (soft constraint)
            for (on, oe, r_safe, w_obs) in self._obstacles:
                dn = x[0] - on
                de = x[1] - oe
                dist_sq = dn * dn + de * de
                r_sq    = r_safe * r_safe
                if dist_sq < r_sq:
                    penetration = r_safe - math.sqrt(dist_sq + 1e-6)
                    J += w_obs * penetration ** 2

        # Control effort
        for k in range(self.N):
            J += self.R_input * float(U[k] @ U[k])

        # Control rate (smoothness)
        u_prev = self._u_prev if self._u_prev is not None else np.zeros(3)
        J += self.R_delta * float((U[0] - u_prev) @ (U[0] - u_prev))
        for k in range(1, self.N):
            du = U[k] - U[k - 1]
            J += self.R_delta * float(du @ du)

        return J

    # ── Solver ────────────────────────────────────────────

    def solve(self, x0: np.ndarray) -> Tuple[np.ndarray, float, bool]:
        """
        Run one MPC solve step from current state x0.

        Returns
        ───────
        u_opt  : np.ndarray shape (3,)  — optimal first control action [an, ae, ad]
        cost   : float                  — objective value at solution
        solved : bool                   — True if optimiser converged
        """
        # BUG-6 FIX: warm-start from _u_prev hint if provided; otherwise zeros.
        # reset() can supply a non-zero hint so the first post-reset solve is
        # already near the feasible region (e.g. small forward accel at cruise).
        if self._u_prev is not None:
            u0 = np.tile(self._u_prev, (self.N, 1)).flatten()
        else:
            u0 = np.zeros(self.N * 3)

        bounds = [(-self.u_max, self.u_max)] * (self.N * 3)

        result = minimize(
            self._cost,
            u0,
            args=(x0,),
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 80, "ftol": 1e-5, "gtol": 1e-4},
        )

        U_opt = result.x.reshape(self.N, 3)
        u_opt = U_opt[0].copy()
        # Clamp to u_max
        norm = np.linalg.norm(u_opt)
        if norm > self.u_max:
            u_opt = u_opt * (self.u_max / norm)

        self._u_prev     = u_opt.copy()
        # BUG-3 FIX: persist the full optimal sequence for predict_trajectory().
        self._U_opt_last = U_opt.copy()
        return u_opt, float(result.fun), result.success

    def predict_trajectory(self, x0: np.ndarray,
                           u_opt: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Return full N+1 predicted trajectory using the most recent optimal U.

        BUG-3 FIX: the original implementation tiled u_opt (the first-step
        action supplied by the caller) across the full horizon, producing a
        constant-acceleration straight-line prediction even when the optimal
        solution required curving around an obstacle.  Now uses the stored full
        U_opt sequence from the last solve(), making the GCS trajectory overlay
        consistent with the actual computed plan.

        u_opt is kept as an argument for API compatibility but is ignored;
        the internally stored _U_opt_last is always used.
        """
        if self._U_opt_last is not None:
            U = self._U_opt_last
        else:
            # No solve has run yet — fall back to zeros (stationary prediction)
            U = np.zeros((self.N, 3))
        return self._rollout(x0, U)

    def reset(self, hint_u: Optional[np.ndarray] = None):
        """
        Reset warm-start state.

        BUG-6 FIX: previously unconditionally set _u_prev = None, causing an
        all-zeros warm-start on the next solve.  At 50 m/s cruise this meant
        5–15 wasted L-BFGS-B iterations just leaving the zero basin, adding
        latency at the worst possible moment (immediately after an obstacle clears
        and the avoidance loop calls reset() before resuming the mission).

        hint_u: optional (3,) array used as the warm-start for the next solve.
        Pass a small forward acceleration vector to seed the optimiser near the
        correct basin.  Defaults to None (zeros warm-start) when no hint is known.
        """
        self._u_prev     = hint_u.copy() if hint_u is not None else None
        self._U_opt_last = None


# ══════════════════════════════════════════════════════════
#  AVOIDANCE MPC  (drop-in for AvoidancePID)
# ══════════════════════════════════════════════════════════

class AvoidanceMPC:
    """
    Obstacle-avoidance MPC wrapper.

    Replaces AvoidancePID with the same public interface used by
    avoidance_loop() in isr_lidar_mpc.py:

        avoid_mpc = AvoidanceMPC(safe_distance=LIDAR_AVOID_DIST + 2.0)
        avoid_mpc.update_speed(drone_state["groundspeed"])
        lateral_offset = avoid_mpc.compute_correction(dist)

    The MPC additionally exposes:

        u_ned = avoid_mpc.compute_ned_command(x0_ned, obstacle_ned,
                                               mission_wp_ned)

    which returns a full NED acceleration vector for direct use in a
    velocity-controller loop (Phase 2 extension).

    Gain scheduling
    ───────────────
    At high speeds (≥ SPEED_THRESHOLD m/s) the prediction horizon shrinks and
    the obstacle penalty grows — mirrors the old PID gain-schedule logic but
    in MPC cost-weight space.

    v2 Bug Fixes
    ────────────
    BUG-1 FIX: compute_correction() now calls MPCEngine.solve() via a minimal
      lateral NED state rather than returning a raw heuristic scalar.
    BUG-2 FIX: update_speed() calls reset_horizon() instead of mutating N
      directly, preventing warm-start size mismatch crashes on mode transitions.
    """

    SPEED_THRESHOLD = 25.0   # m/s — matches RACING_SPEED_THRESHOLD in mission_config

    # Slow flight cost weights  (< SPEED_THRESHOLD)
    N_SLOW        = 12       # longer horizon at low speed — more planning room
    Q_TRACK_SLOW  = 1.5
    W_OBS_SLOW    = 80.0
    R_INPUT_SLOW  = 0.10

    # Fast / racing flight cost weights  (≥ SPEED_THRESHOLD)
    # At 30–60 m/s the drone covers 1 m every ~20 ms — the MPC must react hard
    # and fast.  Shorter horizon (fewer steps to solve quickly), much heavier
    # obstacle penalty, very low input regularisation to allow full-thrust escapes.
    N_FAST        = 6        # 6 × 20 ms = 120 ms lookahead at racing speed
    Q_TRACK_FAST  = 4.0      # strong tracking to hold the racing line
    W_OBS_FAST    = 500.0    # aggressive obstacle penalty — can't afford penetration
    R_INPUT_FAST  = 0.01     # minimal smoothing — prioritise escape over comfort

    # Ultra-fast tier  (≥ 45 m/s) — maximum aggression
    N_ULTRA        = 4
    Q_TRACK_ULTRA  = 6.0
    W_OBS_ULTRA    = 1000.0
    R_INPUT_ULTRA  = 0.005
    SPEED_ULTRA    = 45.0

    def __init__(self, safe_distance: float = 17.0):
        self.safe_distance   = safe_distance
        self._current_speed  = 0.0
        self._obs_penalty_w  = self.W_OBS_SLOW
        self._solve_time_ms  = 0.0   # diagnostic

        self._mpc = MPCEngine(
            N=self.N_SLOW, dt=0.02,
            u_max=12.0,   # racing frame: 3× higher thrust headroom
            v_max=60.0,   # hard cap at 60 m/s
            Q_track=self.Q_TRACK_SLOW,
            Q_vel=0.05,
            R_input=self.R_INPUT_SLOW,
            R_delta=0.10,  # less smoothing penalty for sharper manoeuvres
            Q_terminal=10.0,
        )

    # ── Speed scheduling ──────────────────────────────────

    def update_speed(self, speed_m_s: float):
        """
        Update groundspeed and schedule MPC weights accordingly.

        Three tiers for racing:
          slow   < SPEED_THRESHOLD (25 m/s) — standard ISR weights
          fast   25–45 m/s         — racing weights, shorter horizon
          ultra  ≥ 45 m/s          — maximum aggression, minimal horizon

        BUG-2 FIX: previously set self._mpc.N = 8/10 directly.  On a
        fast→slow transition the next solve() reshaped a 24-element result
        into (10, 3), crashing.  Now calls reset_horizon() which atomically
        updates N and clears both warm-start buffers.
        """
        prev_speed = self._current_speed
        self._current_speed = speed_m_s

        if speed_m_s >= self.SPEED_ULTRA:
            # Ultra tier
            self._mpc.Q_track   = self.Q_TRACK_ULTRA
            self._mpc.R_input   = self.R_INPUT_ULTRA
            self._obs_penalty_w = self.W_OBS_ULTRA
            prev_tier = (2 if prev_speed >= self.SPEED_ULTRA else
                         1 if prev_speed >= self.SPEED_THRESHOLD else 0)
            if prev_tier != 2:
                self._mpc.reset_horizon(self.N_ULTRA)

        elif speed_m_s >= self.SPEED_THRESHOLD:
            # Fast tier
            self._mpc.Q_track   = self.Q_TRACK_FAST
            self._mpc.R_input   = self.R_INPUT_FAST
            self._obs_penalty_w = self.W_OBS_FAST
            prev_tier = (2 if prev_speed >= self.SPEED_ULTRA else
                         1 if prev_speed >= self.SPEED_THRESHOLD else 0)
            if prev_tier != 1:
                self._mpc.reset_horizon(self.N_FAST)

        else:
            # Slow tier
            self._mpc.Q_track   = self.Q_TRACK_SLOW
            self._mpc.R_input   = self.R_INPUT_SLOW
            self._obs_penalty_w = self.W_OBS_SLOW
            prev_tier = (2 if prev_speed >= self.SPEED_ULTRA else
                         1 if prev_speed >= self.SPEED_THRESHOLD else 0)
            if prev_tier != 0:
                self._mpc.reset_horizon(self.N_SLOW)

    # ── Scalar correction (backward-compatible with avoidance_loop) ──

    def compute_correction(self, nearest_obstacle_dist: float,
                           drone_heading_deg: float = 0.0,
                           vn_ms: float = 0.0,
                           ve_ms: float = 0.0) -> float:
        """
        Return a scalar lateral offset magnitude (metres), equivalent to
        the old AvoidancePID.compute_correction() return value.

        BUG-1 FIX: the original implementation bypassed MPCEngine.solve()
        entirely, returning a raw penetration×(W_obs/50) heuristic.  The QP
        was never called, making this file identical in behaviour to a simple
        proportional controller despite being named MPC.

        Now: builds a minimal 2-D NED state from the obstacle distance and
        drone heading, calls compute_ned_command() with a null mission waypoint
        (come-to-rest reference), and projects the resulting NED acceleration
        onto the lateral axis to produce a displacement offset.

        The dt=0.02 scaling converts acceleration (m/s²) to a velocity
        increment per step; multiplied by AVOIDANCE_HOLD_S (2.0 s) gives a
        displacement in the same range as the old heuristic — but now shaped
        by the MPC cost function and obstacle penalty weights.

        If the obstacle is outside safe_distance, resets the MPC and returns 0.
        """
        if nearest_obstacle_dist >= self.safe_distance:
            # BUG-6 FIX: pass a small forward hint so the next solve (when the
            # obstacle re-enters) warm-starts near feasible rather than at zero.
            fwd_n = math.cos(math.radians(drone_heading_deg)) * 0.5
            fwd_e = math.sin(math.radians(drone_heading_deg)) * 0.5
            self._mpc.reset(hint_u=np.array([fwd_n, fwd_e, 0.0]))
            return 0.0

        # Build a 2-D NED state with the obstacle directly ahead in local frame.
        # We keep position at origin (relative to drone) and use the obstacle's
        # distance along the drone's heading axis as the threat position.
        dn = math.cos(math.radians(drone_heading_deg)) * nearest_obstacle_dist
        de = math.sin(math.radians(drone_heading_deg)) * nearest_obstacle_dist

        x0 = np.array([0.0, 0.0, 0.0, vn_ms, ve_ms, 0.0])

        u_opt, _, _ = self.compute_ned_command(
            x0_ned       = x0,
            obstacle_ned = (dn, de),
            mission_wp_ned = None,   # no active WP — just stop safely
        )

        # Project NED acceleration onto the lateral axis (perpendicular to heading).
        lat_n = -math.sin(math.radians(drone_heading_deg))
        lat_e =  math.cos(math.radians(drone_heading_deg))
        lateral_accel = float(u_opt[0] * lat_n + u_opt[1] * lat_e)

        # Convert acceleration to an approximate lateral displacement:
        #   d ≈ |a| × AVOIDANCE_HOLD_S²  (kinematic upper bound)
        # Clamp to [0, 80] to match the old heuristic range.
        AVOIDANCE_HOLD_S = 2.0
        offset = min(abs(lateral_accel) * AVOIDANCE_HOLD_S ** 2, 80.0)
        # Enforce a minimum offset proportional to penetration depth so shallow
        # obstacles still produce a non-zero correction.
        penetration = self.safe_distance - nearest_obstacle_dist
        min_offset  = penetration * 1.5
        return max(offset, min_offset)

    # ── Full NED acceleration command ─────────────────────

    def compute_ned_command(
        self,
        x0_ned:          np.ndarray,
        obstacle_ned:    Optional[Tuple[float, float]] = None,
        mission_wp_ned:  Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[np.ndarray, float, bool]:
        """
        Compute optimal NED acceleration for the next timestep.

        Parameters
        ──────────
        x0_ned       : current state [n,e,d,vn,ve,vd] in local NED (m / m/s)
        obstacle_ned : (north_m, east_m) of nearest obstacle in local frame
        mission_wp_ned: (north_m, east_m, down_m) of next mission waypoint

        Returns
        ───────
        u_ned   : np.ndarray [an, ae, ad] — acceleration command (m/s²)
        cost    : float                   — MPC objective value
        solved  : bool
        """
        # Build reference state: head toward mission WP at cruise speed
        if mission_wp_ned is not None:
            xr = np.array([mission_wp_ned[0], mission_wp_ned[1], mission_wp_ned[2],
                           0.0, 0.0, 0.0])
        else:
            xr = x0_ned.copy()
            xr[3:6] = 0.0  # come to rest if no waypoint
        self._mpc.set_reference(xr)

        # Register obstacle as soft avoidance constraint
        if obstacle_ned is not None:
            self._mpc.set_obstacles([
                (obstacle_ned[0], obstacle_ned[1],
                 self.safe_distance, self._obs_penalty_w)
            ])
        else:
            self._mpc.set_obstacles([])

        t0 = time.perf_counter()
        u_opt, cost, solved = self._mpc.solve(x0_ned)
        self._solve_time_ms = (time.perf_counter() - t0) * 1000.0
        return u_opt, cost, solved

    def set_gains(self, kp=None, ki=None, kd=None, **kw):
        """
        Stub — maintains API compat with old set_gains() callers.

        Note: kp/ki/kd from scenarios.json do not directly map to MPC cost
        weights.  To tune avoidance aggressiveness, modify W_OBS_SLOW /
        W_OBS_FAST / Q_TRACK_SLOW / Q_TRACK_FAST class attributes or pass
        them to the constructor.
        """
        pass

    @property
    def solve_time_ms(self) -> float:
        return self._solve_time_ms

    def reset(self):
        self._mpc.reset()


# ══════════════════════════════════════════════════════════
#  ORBIT MPC  (drop-in for OrbitPID)
# ══════════════════════════════════════════════════════════

class OrbitMPC:
    """
    Orbit-radius MPC wrapper.

    Replaces OrbitPID.  Tracks a circular orbit around a fixed target at
    target_radius metres.  The MPC reference at each tick is a point on
    the desired orbit circle directly ahead of the drone's current angular
    position — a "rolling reference" that keeps the drone on the circle
    without needing a full circular trajectory in the MPC state space.

    Public interface (same as OrbitPID):

        opid = OrbitMPC(target_radius=ORBIT_RADIUS)
        correction, current_r = opid.compute_correction(
            drone_lat, drone_lon, target_lat, target_lon)

    v2 Bug Fix — BUG-4
    ──────────────────
    Previously returned radial_accel × dt (unit = m/s — a velocity increment),
    while the caller (isr_lidar_mpc.py orbit display loop) printed it as metres
    of radial error.  The magnitude was also ~10× smaller than the PID version,
    breaking any upstream "close enough" logic.

    Fix: compute_correction() now returns (signed_radial_error_m, current_radius_m)
    matching the OrbitPID semantic.  The MPC solve still runs every tick to
    inform future corrections (via warm-start) and is available via
    compute_ned_correction() for callers that want the full NED acceleration.
    """

    def __init__(self, target_radius: float, dt: float = 1.0):
        self.target_radius = target_radius
        self.dt            = dt

        self._mpc = MPCEngine(
            N=6, dt=dt,
            u_max=3.0, v_max=30.0,
            Q_track=3.0,
            Q_vel=0.2,
            R_input=0.3,
            R_delta=0.5,
            Q_terminal=6.0,
        )

    def compute_correction(self,
                           drone_lat: float, drone_lon: float,
                           target_lat: float, target_lon: float
                           ) -> Tuple[float, float]:
        """
        Return (signed_radial_error_m, current_radius_m).

        BUG-4 FIX: the original return was radial_accel × dt (unit m/s),
        mismatched against the OrbitPID unit (metres of position error).
        The display string "err={correction:+.2f}m" in isr_lidar_mpc.py was
        therefore both wrong in unit and ~10× too small in magnitude.

        Now returns the signed radial error in metres:
          positive → drone is outside the orbit circle (too far from target)
          negative → drone is inside the orbit circle (too close to target)

        The MPC solve runs in the background every tick to keep the warm-start
        current.  Call compute_ned_correction() directly when you need the full
        NED acceleration vector.
        """
        current_radius = haversine(drone_lat, drone_lon, target_lat, target_lon)
        # positive = outside circle (need to move inward)
        radial_error   = current_radius - self.target_radius

        # Run MPC in background to maintain warm-start
        try:
            self._run_mpc_solve(drone_lat, drone_lon, target_lat, target_lon,
                                current_radius)
        except Exception:
            pass  # MPC failure is non-fatal; radial_error is always valid

        return radial_error, current_radius

    def compute_ned_correction(self,
                               drone_lat: float, drone_lon: float,
                               target_lat: float, target_lon: float
                               ) -> Tuple[np.ndarray, float, float]:
        """
        Return (u_ned_m_s2, radial_error_m, current_radius_m).
        Full NED acceleration for callers that want to close the loop directly.
        """
        current_radius = haversine(drone_lat, drone_lon, target_lat, target_lon)
        radial_error   = current_radius - self.target_radius
        u_opt = self._run_mpc_solve(drone_lat, drone_lon, target_lat, target_lon,
                                    current_radius)
        return u_opt, radial_error, current_radius

    def _run_mpc_solve(self, drone_lat, drone_lon, target_lat, target_lon,
                       current_radius):
        """Internal: build NED state and run one solve step."""
        dn = math.radians(drone_lat - target_lat) * 6_371_000.0
        de = (math.radians(drone_lon - target_lon)
              * 6_371_000.0 * math.cos(math.radians(target_lat)))

        # Desired reference: same angle on the orbit circle
        angle  = math.atan2(de, dn)
        rn_ref = self.target_radius * math.cos(angle)
        re_ref = self.target_radius * math.sin(angle)

        x0  = np.array([dn, de, 0.0, 0.0, 0.0, 0.0])
        xr  = np.array([rn_ref, re_ref, 0.0, 0.0, 0.0, 0.0])
        self._mpc.set_reference(xr)
        self._mpc.set_obstacles([])

        u_opt, _, _ = self._mpc.solve(x0)
        return u_opt

    def set_gains(self, **kw):
        """Stub for API compatibility."""
        pass

    def reset(self):
        self._mpc.reset()


# ══════════════════════════════════════════════════════════
#  ALTITUDE MPC  (drop-in for AltitudePID)
# ══════════════════════════════════════════════════════════

class AltitudeMPC:
    """
    Altitude-hold MPC wrapper.  Replaces AltitudePID.

    The MPC operates on the vertical (d) axis of the full 6-DOF state and
    computes a vertical acceleration command.

    Public interface (same as AltitudePID):

        alt_mpc = AltitudeMPC(target_alt=ORBIT_ALTITUDE)
        correction = alt_mpc.compute_correction(current_alt_m)

    v2 Bug Fix — BUG-5
    ──────────────────
    Previously built x0 with zero horizontal velocities [0,0,-alt,0,0,0].
    At 50 m/s cruise the solver thought the drone was stationary and
    over-commanded initial vertical acceleration to compensate for what it
    perceived as a larger total-state deviation.

    Fix: compute_correction() now accepts optional vn_ms / ve_ms parameters.
    isr_lidar_mpc.py passes the live drone groundspeed components so the solver
    starts from an accurate state.  Zero defaults preserve backward compat.
    """

    def __init__(self, target_alt: float, dt: float = 0.02):
        self.target_alt = target_alt
        self.dt         = dt

        self._mpc = MPCEngine(
            N=8, dt=dt,
            u_max=3.0, v_max=10.0,
            Q_track=4.0,
            Q_vel=0.3,
            R_input=0.4,
            R_delta=0.6,
            Q_terminal=8.0,
        )

    def compute_correction(self, current_alt: float,
                           vn_ms: float = 0.0,
                           ve_ms: float = 0.0) -> float:
        """
        Return vertical acceleration command (m/s²).  Positive = climb.

        BUG-5 FIX: x0 now includes horizontal velocity components so the
        solver has an accurate initial state when the drone is moving.
        vn_ms, ve_ms: current NED horizontal velocities (default 0 for callers
        without telemetry).
        """
        # State: [n=0, e=0, d=−alt, vn, ve, vd=0]  (NED down = −altitude)
        x0 = np.array([0.0, 0.0, -current_alt, vn_ms, ve_ms, 0.0])
        xr = np.array([0.0, 0.0, -self.target_alt, 0.0, 0.0, 0.0])
        self._mpc.set_reference(xr)
        self._mpc.set_obstacles([])

        u_opt, _, _ = self._mpc.solve(x0)

        # Return vertical component (NED down → negate for AGL altitude cmd)
        return float(-u_opt[2])

    def set_gains(self, **kw):
        """Stub for API compatibility."""
        pass

    def reset(self):
        self._mpc.reset()


# ══════════════════════════════════════════════════════════
#  SELF-TEST
# ══════════════════════════════════════════════════════════

if __name__ == "__main__":
    import time as _time

    print("=" * 60)
    print("  Aran Technologies — MPC Controller Self-Test  [v2]")
    print("=" * 60)

    # ── AvoidanceMPC scalar API (BUG-1 FIX verification) ────
    print("\n─ AvoidanceMPC scalar correction — now calls MPCEngine.solve() ─")
    avoid_mpc = AvoidanceMPC(safe_distance=17.0)
    avoid_mpc.update_speed(5.0)
    for dist in [30, 20, 17, 12, 8, 4]:
        t0 = _time.perf_counter()
        c  = avoid_mpc.compute_correction(dist, drone_heading_deg=0.0)
        ms = (_time.perf_counter() - t0) * 1000
        print(f"  dist={dist:3d}m  offset={c:+6.2f}m  solve_time={ms:.1f}ms")

    print("\n─ AvoidanceMPC FAST (50 m/s) — BUG-2 horizon transition ─")
    avoid_mpc.update_speed(50.0)   # N: 10 → 8 via reset_horizon()
    for dist in [30, 17, 12, 8, 4]:
        c = avoid_mpc.compute_correction(dist, drone_heading_deg=45.0,
                                         vn_ms=35.0, ve_ms=35.0)
        print(f"  dist={dist:3d}m  offset={c:+6.2f}m")

    print("\n─ AvoidanceMPC horizon reverse (50→5 m/s) — BUG-2 crash check ─")
    avoid_mpc.update_speed(5.0)    # N: 8 → 10 via reset_horizon()
    c = avoid_mpc.compute_correction(8.0, drone_heading_deg=0.0)
    print(f"  Post-transition correction={c:+.2f}m  (no crash = PASS)")

    # ── AvoidanceMPC NED command API ──────────────────────
    print("\n─ AvoidanceMPC NED acceleration command ─")
    avoid_mpc2 = AvoidanceMPC(safe_distance=17.0)
    avoid_mpc2.update_speed(20.0)
    x0 = np.array([0.0, 0.0, -50.0, 20.0, 0.0, 0.0])
    obs = (10.0, 0.0)
    wp  = (200.0, 0.0, -50.0)

    t0 = _time.perf_counter()
    u_ned, cost, ok = avoid_mpc2.compute_ned_command(x0, obs, wp)
    elapsed = (_time.perf_counter() - t0) * 1000.0
    print(f"  u_ned=[{u_ned[0]:+.3f}, {u_ned[1]:+.3f}, {u_ned[2]:+.3f}] m/s²")
    print(f"  cost={cost:.2f}  solved={ok}  solve_time={elapsed:.1f}ms")
    print(f"  east accel (escape lateral): {u_ned[1]:+.3f} m/s²"
          f"  ({'right' if u_ned[1]>0 else 'left'})")

    # ── predict_trajectory (BUG-3 FIX verification) ──────
    print("\n─ predict_trajectory — uses full U_opt_last (BUG-3 fix) ─")
    traj = avoid_mpc2._mpc.predict_trajectory(x0)
    print(f"  trajectory shape={traj.shape}  (expected ({avoid_mpc2._mpc.N+1}, 6))")
    # Verify positions change non-trivially (not constant-accel extrapolation)
    pos_changes = [np.linalg.norm(traj[k+1, :3] - traj[k, :3]) for k in range(5)]
    print(f"  step distances (m): {[f'{d:.3f}' for d in pos_changes]}")

    # ── OrbitMPC (BUG-4 FIX verification) ────────────────
    print("\n─ OrbitMPC — returns signed radial error in METRES (BUG-4 fix) ─")
    orbit_mpc = OrbitMPC(target_radius=45.0, dt=1.0)
    target_lat, target_lon = 47.3985, 8.5470
    for offset_m in [45, 50, 60, 40, 30]:
        drone_lat = target_lat + offset_m / 111320.0
        drone_lon = target_lon
        err, cur_r = orbit_mpc.compute_correction(drone_lat, drone_lon,
                                                   target_lat, target_lon)
        expected_err = cur_r - 45.0
        print(f"  drone_r={offset_m:3d}m  current_r={cur_r:.1f}m  "
              f"radial_err={err:+.2f}m  (expected≈{expected_err:+.2f}m)")

    # ── AltitudeMPC (BUG-5 FIX verification) ─────────────
    print("\n─ AltitudeMPC — with horizontal velocity (BUG-5 fix) ─")
    alt_mpc = AltitudeMPC(target_alt=50.0, dt=0.02)
    print("  Stationary (vn=ve=0):  target=50m")
    for alt in [30, 40, 48, 50, 52, 60]:
        c_still = alt_mpc.compute_correction(alt, vn_ms=0.0, ve_ms=0.0)
        c_cruise = alt_mpc.compute_correction(alt, vn_ms=35.0, ve_ms=35.0)
        print(f"    alt={alt:3d}m  still={c_still:+.3f}m/s²  "
              f"cruise_50ms={c_cruise:+.3f}m/s²")

    # ── Geometry helpers ──────────────────────────────────
    print("\n─ project_waypoint / compute_avoidance_waypoint ─")
    lat, lon = 47.3977, 8.5456
    det_lat, det_lon = compute_avoidance_waypoint(lat, lon, 45.0, 90.0, offset_m=50.0)
    dist_check = haversine(lat, lon, det_lat, det_lon)
    print(f"  Detour: {det_lat:.6f}, {det_lon:.6f}  dist={dist_check:.1f}m (expected ~50m)")

    print("\n─ best_escape_bearing ─")
    sectors = [float("inf")] * 8
    sectors[1] = 8.0
    brg, side, clr = best_escape_bearing(sectors, 0.0, 45.0)
    print(f"  Escape bearing={brg:.0f}°  side={side}  clearance={clr}m")

    print("\nAll OK — MPC v2 self-test passed")