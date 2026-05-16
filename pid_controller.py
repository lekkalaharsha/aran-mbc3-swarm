"""
Aran Technologies — PID Controller Module  [v11]
Three controllers: AvoidancePID, OrbitPID, AltitudePID
Used by isr_lidar_pid.py

v11 Bug Fix:
  - PIDController.set_gains(): _Tt was only recomputed inside the `if ki`
    branch; a kp-only update (e.g. /pid_tune sending kp without ki) left _Tt
    stale, producing wrong anti-windup back-calculation strength. Now _Tt is
    always recomputed when either kp or ki changes.

v10 Bug Fixes:
  - best_escape_bearing(): obstacle_bearing_deg is already absolute from
    _bearing_to_nearest(); removed spurious re-addition of drone_heading_deg
    which produced a ~90deg error at non-zero headings
  - PIDController.compute(): back-calculation anti-windup formula corrected —
    removed erroneous `* dt` factor; wind-back is now Tt-scaled only (standard
    Åström formula), restoring correct integral damping at 50 Hz

v9 Changes:
  - Back-calculation anti-windup (replaces simple clamp)
  - Gain scheduling in AvoidancePID — tighter gains at high speed
  - project_waypoint() — direct bearing projection (replaces perpendicular hack)
  - compute_avoidance_waypoint() fixed to use direct escape projection
  - best_escape_bearing() — chooses left/right intelligently vs always +90 deg
  - haversine() consolidated here; import from this module everywhere
  - All controllers expose set_gains() for live tuning via /pid_tune endpoint
"""
import time
import math


# ══════════════════════════════════════════════════════════
#  SHARED GEOMETRY
# ══════════════════════════════════════════════════════════

def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two lat/lon points."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def project_waypoint(lat, lon, bearing_deg, distance_m):
    """
    Project a point from (lat, lon) along bearing_deg for distance_m metres.
    bearing_deg: 0=North, clockwise positive.
    Returns (new_lat, new_lon).
    """
    R = 6371000
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    d = distance_m / R
    lat2 = math.asin(
        math.sin(lat1)*math.cos(d) +
        math.cos(lat1)*math.sin(d)*math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing)*math.sin(d)*math.cos(lat1),
        math.cos(d) - math.sin(lat1)*math.sin(lat2)
    )
    return math.degrees(lat2), math.degrees(lon2)


def compute_avoidance_waypoint(drone_lat, drone_lon, drone_heading_deg,
                                escape_bearing_deg, offset_m=20.0):
    """
    Compute a detour waypoint by projecting directly along escape_bearing_deg.

    v9 FIX: Previously applied a +90deg perpendicular rotation on top of the
    already-absolute escape bearing from sector analysis, causing the drone to
    fly ~90deg off the intended escape path. Now we project directly along the
    escape bearing — no extra rotation.

    drone_heading_deg  : kept for API compatibility (unused)
    escape_bearing_deg : absolute world bearing toward the clearest escape sector
    offset_m           : detour distance in metres
    """
    return project_waypoint(drone_lat, drone_lon, escape_bearing_deg, offset_m)


def best_escape_bearing(sectors, drone_heading_deg, obstacle_bearing_deg):
    """
    Choose escape direction intelligently: left or right of the obstacle,
    whichever sector has the most clearance.

    sectors             : list of 8 min-distances per sector (inf=clear)
    drone_heading_deg   : current drone heading (absolute, unused — kept for API compat)
    obstacle_bearing_deg: ABSOLUTE world bearing to nearest obstacle
                          (_bearing_to_nearest already returns 0-360 absolute degrees)

    Returns (absolute_bearing_deg, side_str, clearance_m)

    BUG FIX: Previously computed `abs_obstacle = drone_heading + obstacle_bearing`,
    double-adding the heading since _bearing_to_nearest already returns an absolute
    bearing. At a heading of 90° a dead-ahead obstacle (90° abs) would resolve to
    180° — wrong escape sector. Now obstacle_bearing_deg is used directly.
    """
    sector_count = len(sectors)
    sector_size  = 360.0 / sector_count
    abs_obstacle = obstacle_bearing_deg % 360   # already absolute

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
#  BASE PID — back-calculation anti-windup
# ══════════════════════════════════════════════════════════

class PIDController:
    """
    Standard PID with back-calculation anti-windup.

    When output saturates the integral is wound back by:
      (saturated - unsaturated) / (ki * Tt) * dt
    where Tt = sqrt(kp/ki). This prevents windup accumulation at the
    50m/s cruise speed where errors can be large and persistent.
    """
    def __init__(self, kp, ki, kd, output_limit=10.0, name="PID"):
        self.kp           = kp
        self.ki           = ki
        self.kd           = kd
        self.output_limit = output_limit
        self.name         = name
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._prev_time   = None
        self._initialized = False
        self._Tt = math.sqrt(kp / ki) if ki > 0 else 1.0

    def set_gains(self, kp=None, ki=None, kd=None, output_limit=None):
        """Live gain update — used by /pid_tune endpoint and gain scheduling.

        BUG FIX: previously _Tt was only recomputed inside the `if ki` branch,
        so a kp-only update (e.g. /pid_tune sending just kp) left _Tt computed
        from the old kp, producing wrong anti-windup damping.  Now _Tt is
        always recomputed whenever kp or ki changes.
        """
        if kp           is not None: self.kp = kp
        if ki           is not None: self.ki = ki
        if kd           is not None: self.kd = kd
        if output_limit is not None: self.output_limit = output_limit
        # Recompute Tt any time kp or ki may have changed
        if kp is not None or ki is not None:
            self._Tt = math.sqrt(self.kp / self.ki) if self.ki > 0 else 1.0

    def reset(self):
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._prev_time   = None
        self._initialized = False

    def compute(self, setpoint, measured):
        now   = time.time()
        error = setpoint - measured

        if not self._initialized:
            self._prev_error  = error
            self._prev_time   = now
            self._initialized = True
            return 0.0

        dt = now - self._prev_time
        if dt <= 0.0:
            return 0.0

        p        = self.kp * error
        self._integral += error * dt
        i_raw    = self.ki * self._integral
        d        = self.kd * (error - self._prev_error) / dt

        raw_output = p + i_raw + d
        clamped    = max(-self.output_limit, min(self.output_limit, raw_output))

        # Back-calculation anti-windup
        # BUG FIX: removed spurious `* dt` — standard formula divides by Tt only,
        # not Tt*dt. The extra dt factor made wind-back ~50x too weak at 50 Hz.
        if self.ki > 0 and raw_output != clamped:
            self._integral -= (raw_output - clamped) / (self.ki * self._Tt)

        self._prev_error = error
        self._prev_time  = now
        return clamped


# ══════════════════════════════════════════════════════════
#  SPECIALIZED CONTROLLERS
# ══════════════════════════════════════════════════════════

class OrbitPID:
    def __init__(self, target_radius, kp=0.8, ki=0.05, kd=0.3):
        self.target_radius = target_radius
        self.pid = PIDController(kp=kp, ki=ki, kd=kd,
                                  output_limit=5.0, name="OrbitRadius")

    def compute_correction(self, drone_lat, drone_lon, target_lat, target_lon):
        current_radius = haversine(drone_lat, drone_lon, target_lat, target_lon)
        correction = self.pid.compute(setpoint=self.target_radius,
                                       measured=current_radius)
        return correction, current_radius

    def set_gains(self, **kw): self.pid.set_gains(**kw)
    def reset(self):           self.pid.reset()


class AltitudePID:
    def __init__(self, target_alt, kp=1.2, ki=0.1, kd=0.4):
        self.target_alt = target_alt
        self.pid = PIDController(kp=kp, ki=ki, kd=kd,
                                  output_limit=3.0, name="Altitude")

    def compute_correction(self, current_alt):
        return self.pid.compute(setpoint=self.target_alt, measured=current_alt)

    def set_gains(self, **kw): self.pid.set_gains(**kw)
    def reset(self):           self.pid.reset()


class AvoidancePID:
    """
    v9: Gain scheduling based on drone groundspeed.

    At 50m/s a 15m obstacle gives only 300ms reaction time.
    Aggressive gains (Kp=2.8) produce larger, faster lateral offsets.
    At low speeds soft gains (Kp=1.5) prevent oscillation and overshoot.
    """
    SPEED_THRESHOLD = 20.0   # m/s threshold for gain switching

    KP_SLOW, KI_SLOW, KD_SLOW = 1.5, 0.0, 0.6   # hover / approach
    KP_FAST, KI_FAST, KD_FAST = 2.8, 0.0, 0.9   # 50 m/s cruise

    def __init__(self, safe_distance=8.0):
        self.safe_distance   = safe_distance
        self._current_speed  = 0.0
        self.pid = PIDController(kp=self.KP_SLOW, ki=self.KI_SLOW,
                                  kd=self.KD_SLOW, output_limit=15.0,
                                  name="Avoidance")

    def update_speed(self, speed_m_s):
        """Update groundspeed and schedule gains accordingly."""
        self._current_speed = speed_m_s
        if speed_m_s >= self.SPEED_THRESHOLD:
            self.pid.set_gains(kp=self.KP_FAST, ki=self.KI_FAST, kd=self.KD_FAST)
        else:
            self.pid.set_gains(kp=self.KP_SLOW, ki=self.KI_SLOW, kd=self.KD_SLOW)

    def compute_correction(self, nearest_obstacle_dist):
        if nearest_obstacle_dist >= self.safe_distance:
            self.pid.reset()
            return 0.0
        return self.pid.compute(setpoint=self.safe_distance,
                                 measured=nearest_obstacle_dist)

    def set_gains(self, **kw): self.pid.set_gains(**kw)
    def reset(self):           self.pid.reset()


# ══════════════════════════════════════════════════════════
#  SELF-TEST
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Testing PID controllers (v9)...")

    print("\n─ AltitudePID ─")
    alt_pid = AltitudePID(target_alt=50.0)
    for alt in [45, 47, 49, 50, 51, 50.2]:
        c = alt_pid.compute_correction(alt)
        print(f"  Alt={alt:.1f}m  correction={c:+.3f}m/s")

    print("\n─ AvoidancePID slow (5 m/s) ─")
    avoid_slow = AvoidancePID(safe_distance=8.0)
    avoid_slow.update_speed(5.0)
    for dist in [10, 7, 5, 3, 2]:
        c = avoid_slow.compute_correction(dist)
        print(f"  Obstacle={dist}m  offset={c:+.2f}m")

    print("\n─ AvoidancePID fast (50 m/s) ─")
    avoid_fast = AvoidancePID(safe_distance=8.0)
    avoid_fast.update_speed(50.0)
    for dist in [10, 7, 5, 3, 2]:
        c = avoid_fast.compute_correction(dist)
        print(f"  Obstacle={dist}m  offset={c:+.2f}m")

    print("\n─ project_waypoint / compute_avoidance_waypoint ─")
    lat, lon = 47.3977, 8.5456
    det_lat, det_lon = compute_avoidance_waypoint(lat, lon, 45.0, 90.0, offset_m=50.0)
    dist_check = haversine(lat, lon, det_lat, det_lon)
    print(f"  Detour: {det_lat:.6f}, {det_lon:.6f}  dist={dist_check:.1f}m (expected ~50m)")

    print("\n─ best_escape_bearing ─")
    sectors = [float('inf')]*8
    sectors[1] = 8.0
    brg, side, clr = best_escape_bearing(sectors, 0.0, 45.0)
    print(f"  Escape bearing={brg:.0f}°  side={side}  clearance={clr}m")

    print("\nAll OK!")