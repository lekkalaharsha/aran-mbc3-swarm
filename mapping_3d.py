"""
Aran Technologies — 3D LiDAR Mapping Module  [v1]
Accumulates per-scan point clouds and builds an incremental voxel occupancy
grid from 360° LiDAR data + drone pose telemetry.

Architecture
────────────
  PointCloudAccumulator
      Converts each LiDAR scan (range array + metadata) and the current drone
      pose (lat, lon, alt, heading) into world-frame Cartesian (x, y, z) points
      anchored at HOME_LAT / HOME_LON.  Points are stored as a flat NumPy array
      for efficient downstream consumption.  Falls back to a pure-Python list
      path when NumPy is not installed.

  VoxelGrid
      Bins accumulated points into a 3D occupancy grid with configurable cell
      resolution (MAP_RESOLUTION_M).  Each voxel stores an occupancy count.
      Provides:
        • get_slice_2d(alt_min, alt_max)  — horizontal cross-section for GCS
        • get_stats()                     — summary dict for /map_stats endpoint
        • to_pcd_ascii()                  — full ASCII PCD string for file export
        • to_geojson_slice(alt_min, alt_max) — GeoJSON for Leaflet overlay

  save_pcd(path, accumulator)
      Write the accumulated cloud to a standard ASCII .pcd file.

Integration points
──────────────────
  isr_lidar_pid.py  — call accumulator.ingest() in lidar_gz_reader() and
                      lidar_sim_reader() after each scan; call save_pcd() at
                      RTL/mission complete; push map_stats to GCS via push_to_gcs().

  telemetry_web_13.py — POST /map_update receives voxel stats from isr_lidar_pid;
                         GET  /map_slice  returns a 2D GeoJSON slice at the
                         drone's current altitude ± MAP_SLICE_BAND_M.

Coordinate system
─────────────────
  World-frame Cartesian, origin at (HOME_LAT, HOME_LON, 0 m AGL).
    +X  →  East   (metres)
    +Y  →  North  (metres)
    +Z  →  Up     (metres AGL)

  A 360° horizontal LiDAR scan produces points at the drone's current Z.
  Each range ray i is converted:
      sensor_angle_rad = angle_min + i * angle_increment
      world_angle_rad  = sensor_angle_rad + heading_rad          # rotate to North ref
      px = drone_x + range_i * sin(world_angle_rad)              # East
      py = drone_y + range_i * cos(world_angle_rad)              # North
      pz = drone_z                                               # altitude of scan plane

  Drone position in Cartesian:
      drone_x = haversine_east (lon delta × cos(lat) × R)
      drone_y = haversine_north (lat delta × R)
      drone_z = rel_altitude_m

No-fly zone walls and obstacle detections are all included — the voxel grid is
an unfiltered occupancy map of whatever the LiDAR sees.
"""

import math
import os
import time
import threading

# ──────────────────────────────────────────────────────────
#  CONFIG  (overridable by importing module before first use)
# ──────────────────────────────────────────────────────────
try:
    from mission_config import HOME_LAT, HOME_LON, MAP_RESOLUTION_M, MAP_SAVE_PATH
except ImportError:
    HOME_LAT         = 47.3977
    HOME_LON         = 8.5456
    MAP_RESOLUTION_M = 1.0     # metres per voxel cell
    MAP_SAVE_PATH    = "map_output"

MAP_SLICE_BAND_M  = 5.0    # ± metres around drone altitude for 2D slice
MAX_RANGE_M       = 60.0   # discard rays beyond this (noise / ground returns)
MIN_RANGE_M       = 0.15   # discard rays closer than this (self-hit)

# ──────────────────────────────────────────────────────────
#  OPTIONAL NUMPY FAST PATH
# ──────────────────────────────────────────────────────────
try:
    import numpy as np
    _NUMPY = True
except ImportError:
    _NUMPY = False

# ──────────────────────────────────────────────────────────
#  GEOMETRY HELPERS
# ──────────────────────────────────────────────────────────
_R_EARTH = 6371000.0


def _latlon_to_xy(lat, lon):
    """
    Convert (lat, lon) to (x_east_m, y_north_m) relative to (HOME_LAT, HOME_LON).
    Uses equirectangular approximation — accurate to <1 mm for the survey areas
    in mission_config (grid spans ~100 m).
    """
    dlat = math.radians(lat - HOME_LAT)
    dlon = math.radians(lon - HOME_LON)
    x = dlon * math.cos(math.radians(HOME_LAT)) * _R_EARTH   # East
    y = dlat * _R_EARTH                                        # North
    return x, y


def _scan_to_points(drone_x, drone_y, drone_z,
                    heading_rad,
                    ranges,
                    angle_min, angle_increment):
    """
    Convert a LiDAR range array to world-frame (x, y, z) point list.

    drone_x, drone_y  : metres East/North from HOME
    drone_z           : metres AGL
    heading_rad       : drone heading in radians (0 = North, clockwise)
    ranges            : list/array of range values in metres
    angle_min         : sensor-frame start angle (radians)
    angle_increment   : radians per ray

    Returns list of (x, y, z) tuples — invalid rays are dropped.
    """
    points = []
    if _NUMPY:
        idxs = np.arange(len(ranges))
        r    = np.asarray(ranges, dtype=float)
        valid = (r > MIN_RANGE_M) & (r < MAX_RANGE_M) & np.isfinite(r)
        r     = r[valid]
        idxs  = idxs[valid]
        if len(r) == 0:
            return points
        sensor_angles = angle_min + idxs * angle_increment
        world_angles  = sensor_angles + heading_rad        # rotate to world frame
        xs = drone_x + r * np.sin(world_angles)
        ys = drone_y + r * np.cos(world_angles)
        zs = np.full_like(r, drone_z)
        points = list(zip(xs.tolist(), ys.tolist(), zs.tolist()))
    else:
        for i, r in enumerate(ranges):
            if r <= MIN_RANGE_M or r >= MAX_RANGE_M or math.isinf(r) or math.isnan(r):
                continue
            sensor_a = angle_min + i * angle_increment
            world_a  = sensor_a + heading_rad
            px = drone_x + r * math.sin(world_a)
            py = drone_y + r * math.cos(world_a)
            points.append((px, py, drone_z))
    return points


# ══════════════════════════════════════════════════════════
#  POINT CLOUD ACCUMULATOR
# ══════════════════════════════════════════════════════════

class PointCloudAccumulator:
    """
    Thread-safe accumulator of world-frame (x, y, z) points from LiDAR scans.

    Usage:
        acc = PointCloudAccumulator()
        # Inside lidar_gz_reader / lidar_sim_reader, after each scan:
        acc.ingest(ranges, angle_min, angle_increment, drone_state)

    acc.points() returns the current list of (x, y, z) tuples.
    acc.reset()  clears all accumulated points (e.g. on new mission start).
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._pts    = []          # list of (x, y, z)
        self._count  = 0           # number of scans ingested
        self._t_last = None

    # ------------------------------------------------------------------
    def ingest(self, ranges, angle_min, angle_increment, drone_state):
        """
        Convert one LiDAR scan + drone pose into world-frame points and append.

        drone_state must contain:
            lat, lon        : degrees
            alt             : metres AGL (relative)
            heading         : degrees (0 = North, clockwise)

        Invalid / out-of-range rays are silently dropped.
        """
        lat     = drone_state.get("lat",     HOME_LAT)
        lon     = drone_state.get("lon",     HOME_LON)
        alt     = drone_state.get("alt",     0.0)
        heading = drone_state.get("heading", 0.0)

        dx, dy      = _latlon_to_xy(lat, lon)
        heading_rad = math.radians(heading)

        new_pts = _scan_to_points(dx, dy, alt, heading_rad,
                                   ranges, angle_min, angle_increment)
        if not new_pts:
            return

        with self._lock:
            self._pts.extend(new_pts)
            self._count += 1
            self._t_last = time.time()

    # ------------------------------------------------------------------
    def points(self):
        """Return a snapshot copy of accumulated (x, y, z) points."""
        with self._lock:
            return list(self._pts)

    def reset(self):
        with self._lock:
            self._pts.clear()
            self._count  = 0
            self._t_last = None

    @property
    def scan_count(self):
        return self._count

    @property
    def point_count(self):
        with self._lock:
            return len(self._pts)


# ══════════════════════════════════════════════════════════
#  VOXEL GRID
# ══════════════════════════════════════════════════════════

class VoxelGrid:
    """
    3-D occupancy grid built from accumulated point cloud data.

    Voxel key = (ix, iy, iz) where:
        ix = floor(x / resolution)
        iy = floor(y / resolution)
        iz = floor(z / resolution)

    Each voxel stores an integer hit count.  To avoid unbounded memory growth
    during long missions, voxels with counts > VOXEL_MAX_HITS are capped.

    Thread-safe: all mutations hold self._lock.

    Usage:
        grid = VoxelGrid(resolution_m=1.0)
        grid.ingest_points(accumulator.points())
        stats = grid.get_stats()
        slice_data = grid.get_slice_2d(alt_min=45, alt_max=55)
        geojson = grid.to_geojson_slice(alt_min=45, alt_max=55)
        pcd_str = grid.to_pcd_ascii()
    """

    VOXEL_MAX_HITS = 255

    def __init__(self, resolution_m=None):
        self.resolution = resolution_m if resolution_m is not None else MAP_RESOLUTION_M
        self._lock      = threading.Lock()
        self._grid      = {}     # {(ix, iy, iz): hit_count}
        self._last_ingest_count = 0   # accumulator.point_count watermark

    # ------------------------------------------------------------------
    def _key(self, x, y, z):
        ix = int(math.floor(x / self.resolution))
        iy = int(math.floor(y / self.resolution))
        iz = int(math.floor(z / self.resolution))
        return ix, iy, iz

    def ingest_points(self, points):
        """Add a list of (x, y, z) points to the voxel grid."""
        with self._lock:
            for x, y, z in points:
                k = self._key(x, y, z)
                c = self._grid.get(k, 0)
                if c < self.VOXEL_MAX_HITS:
                    self._grid[k] = c + 1

    def clear(self):
        with self._lock:
            self._grid.clear()

    # ------------------------------------------------------------------
    def get_stats(self):
        """Return a summary dict suitable for JSON serialisation."""
        with self._lock:
            n = len(self._grid)
            if n == 0:
                return {
                    "voxel_count": 0,
                    "resolution_m": self.resolution,
                    "bounds": None,
                    "alt_range_m": None,
                }
            keys = list(self._grid.keys())
            xs   = [k[0] for k in keys]
            ys   = [k[1] for k in keys]
            zs   = [k[2] for k in keys]
            r    = self.resolution
            return {
                "voxel_count":  n,
                "resolution_m": r,
                "bounds": {
                    "x_min_m": min(xs) * r, "x_max_m": (max(xs) + 1) * r,
                    "y_min_m": min(ys) * r, "y_max_m": (max(ys) + 1) * r,
                    "z_min_m": min(zs) * r, "z_max_m": (max(zs) + 1) * r,
                },
                "alt_range_m": {
                    "min": round(min(zs) * r, 1),
                    "max": round((max(zs) + 1) * r, 1),
                },
            }

    # ------------------------------------------------------------------
    def get_slice_2d(self, alt_min, alt_max):
        """
        Return all occupied voxels whose Z-band overlaps [alt_min, alt_max].

        Returns list of dicts:
            {"x_m": float, "y_m": float, "z_m": float, "hits": int}
        where x_m, y_m, z_m are the voxel-centre coordinates in metres from HOME.

        Used by /map_slice endpoint to feed the GCS Leaflet overlay.
        """
        r = self.resolution
        iz_min = int(math.floor(alt_min / r))
        iz_max = int(math.floor(alt_max / r))
        result = []
        with self._lock:
            for (ix, iy, iz), hits in self._grid.items():
                if iz_min <= iz <= iz_max:
                    result.append({
                        "x_m":  (ix + 0.5) * r,
                        "y_m":  (iy + 0.5) * r,
                        "z_m":  (iz + 0.5) * r,
                        "hits": hits,
                    })
        return result

    # ------------------------------------------------------------------
    def to_geojson_slice(self, alt_min, alt_max):
        """
        Return a GeoJSON FeatureCollection of 2D voxel squares for the Leaflet
        overlay.  Each feature is a Polygon covering one voxel cell in lat/lon.

        Coordinate transform inverts _latlon_to_xy():
            lat = HOME_LAT + y_m / R_EARTH   (degrees)
            lon = HOME_LON + x_m / (R_EARTH * cos(HOME_LAT_rad))

        Feature properties:
            hits      : occupancy count
            z_m       : voxel-centre altitude
            intensity : normalised hit count 0–1 (for colour mapping)
        """
        slice_voxels = self.get_slice_2d(alt_min, alt_max)
        if not slice_voxels:
            return {"type": "FeatureCollection", "features": []}

        max_hits  = max(v["hits"] for v in slice_voxels) or 1
        r         = self.resolution
        cos_lat   = math.cos(math.radians(HOME_LAT))
        features  = []

        for v in slice_voxels:
            xc, yc = v["x_m"], v["y_m"]
            # voxel corners in metres
            x0, x1 = xc - r / 2, xc + r / 2
            y0, y1 = yc - r / 2, yc + r / 2
            # convert corners to lat/lon
            def to_ll(x, y):
                lat = HOME_LAT + y / _R_EARTH * (180 / math.pi)
                lon = HOME_LON + x / (_R_EARTH * cos_lat) * (180 / math.pi)
                return [round(lon, 8), round(lat, 8)]

            coords = [
                to_ll(x0, y0), to_ll(x1, y0),
                to_ll(x1, y1), to_ll(x0, y1),
                to_ll(x0, y0),   # close ring
            ]
            features.append({
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [coords]},
                "properties": {
                    "hits":      v["hits"],
                    "z_m":       round(v["z_m"], 1),
                    "intensity": round(v["hits"] / max_hits, 3),
                },
            })

        return {"type": "FeatureCollection", "features": features}

    # ------------------------------------------------------------------
    def to_pcd_ascii(self):
        """
        Serialise the voxel CENTRES to a standard ASCII PCD format string.

        PCD fields: x y z intensity
            x, y, z    — voxel-centre coordinates in metres from HOME (ENU)
            intensity  — occupancy hit count (uint8)

        The output can be written to a .pcd file and opened in CloudCompare,
        PCL viewer, Open3D, or RViz.
        """
        with self._lock:
            pts = [
                ((ix + 0.5) * self.resolution,
                 (iy + 0.5) * self.resolution,
                 (iz + 0.5) * self.resolution,
                 hits)
                for (ix, iy, iz), hits in self._grid.items()
            ]

        n = len(pts)
        if n == 0:
            return ""

        lines = [
            "# .PCD v0.7 — Aran Technologies 3D Occupancy Map",
            "# Generated by mapping_3d.py",
            f"# Resolution: {self.resolution}m  Origin: ({HOME_LAT}, {HOME_LON})",
            "# Frame: ENU — +X East, +Y North, +Z Up (metres from HOME)",
            "VERSION 0.7",
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            f"WIDTH {n}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {n}",
            "DATA ascii",
        ]
        for x, y, z, hits in pts:
            lines.append(f"{x:.3f} {y:.3f} {z:.3f} {hits}")
        return "\n".join(lines) + "\n"


# ══════════════════════════════════════════════════════════
#  FILE I/O
# ══════════════════════════════════════════════════════════

def save_pcd(path, accumulator_or_grid, label="map"):
    """
    Write a .pcd file from either a PointCloudAccumulator or a VoxelGrid.

    If a PointCloudAccumulator is passed, writes raw point cloud (one point per
    valid LiDAR ray).  If a VoxelGrid is passed, writes voxel-centre points.

    path  : directory (created if needed) or full file path ending in .pcd
    label : file name prefix when path is a directory

    Returns the full file path written.
    """
    os.makedirs(path, exist_ok=True) if not path.endswith(".pcd") else None

    if path.endswith(".pcd"):
        file_path = path
    else:
        ts        = time.strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(path, f"{label}_{ts}.pcd")

    if isinstance(accumulator_or_grid, VoxelGrid):
        content = accumulator_or_grid.to_pcd_ascii()
    else:
        # Raw point cloud from PointCloudAccumulator
        pts = accumulator_or_grid.points()
        n   = len(pts)
        if n == 0:
            return None
        lines = [
            "# .PCD v0.7 — Aran Technologies Raw LiDAR Point Cloud",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {n}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {n}",
            "DATA ascii",
        ]
        for x, y, z in pts:
            lines.append(f"{x:.3f} {y:.3f} {z:.3f}")
        content = "\n".join(lines) + "\n"

    if not content:
        return None

    with open(file_path, "w") as f:
        f.write(content)

    return file_path


# ══════════════════════════════════════════════════════════
#  INCREMENTAL UPDATE HELPER
# ══════════════════════════════════════════════════════════

class MapBuilder:
    """
    Convenience wrapper that owns both an accumulator and a voxel grid and
    provides a single ingest() call for isr_lidar_pid.py.

    Usage:
        map_builder = MapBuilder()

        # In lidar loop (after lidar_state update):
        map_builder.ingest(ranges, angle_min, angle_increment, drone_state)

        # In push_to_gcs():
        stats = map_builder.stats()      # for /map_update POST

        # At RTL:
        paths = map_builder.save(MAP_SAVE_PATH)
    """

    def __init__(self, resolution_m=None):
        self.accumulator = PointCloudAccumulator()
        self.grid        = VoxelGrid(resolution_m=resolution_m)
        self._ingest_lock = threading.Lock()

    def ingest(self, ranges, angle_min, angle_increment, drone_state):
        """
        Ingest one LiDAR scan.  Converts to points, appends to accumulator,
        and immediately voxelises into the grid.
        """
        with self._ingest_lock:
            lat     = drone_state.get("lat",     HOME_LAT)
            lon     = drone_state.get("lon",     HOME_LON)
            alt     = drone_state.get("alt",     0.0)
            heading = drone_state.get("heading", 0.0)

            dx, dy      = _latlon_to_xy(lat, lon)
            heading_rad = math.radians(heading)

            pts = _scan_to_points(dx, dy, alt, heading_rad,
                                   ranges, angle_min, angle_increment)
            if pts:
                # BUG-E FIX: accumulator._pts and _count were mutated without
                # holding accumulator._lock, creating a race with points() and
                # reset() which both acquire that lock.  Hold it explicitly here.
                with self.accumulator._lock:
                    self.accumulator._pts.extend(pts)
                    self.accumulator._count += 1
                self.grid.ingest_points(pts)

    def stats(self):
        """Return voxel grid stats + point/scan counts for GCS push."""
        s = self.grid.get_stats()
        s["raw_point_count"] = self.accumulator.point_count
        s["scan_count"]      = self.accumulator.scan_count
        return s

    def geojson_slice(self, drone_alt):
        """GeoJSON for Leaflet overlay at drone's current altitude ± band."""
        return self.grid.to_geojson_slice(
            drone_alt - MAP_SLICE_BAND_M,
            drone_alt + MAP_SLICE_BAND_M,
        )

    def save(self, directory=None):
        """
        Save both raw PCD and voxel PCD to directory.
        Returns dict {"raw_pcd": path, "voxel_pcd": path}.
        """
        d = directory or MAP_SAVE_PATH
        os.makedirs(d, exist_ok=True)
        raw_path   = save_pcd(d, self.accumulator, label="raw_cloud")
        voxel_path = save_pcd(d, self.grid,        label="voxel_map")
        return {"raw_pcd": raw_path, "voxel_pcd": voxel_path}

    def reset(self):
        self.accumulator.reset()
        self.grid.clear()


# ══════════════════════════════════════════════════════════
#  SELF-TEST
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    import random

    print("mapping_3d.py — self-test")
    print(f"  NumPy: {'available' if _NUMPY else 'not installed — pure Python path'}")
    print(f"  Resolution: {MAP_RESOLUTION_M}m")

    builder = MapBuilder(resolution_m=1.0)

    # Simulate 5 scans from two drone positions
    scenarios = [
        {"lat": HOME_LAT, "lon": HOME_LON, "alt": 50.0, "heading": 0.0},
        {"lat": HOME_LAT + 0.0001, "lon": HOME_LON + 0.0001, "alt": 52.0, "heading": 45.0},
    ]
    for pose in scenarios:
        # 360-ray scan with some obstacles at 12m
        ranges = [float("inf")] * 360
        for i in range(40, 55):
            ranges[i] = 12.0 + random.uniform(-0.3, 0.3)
        builder.ingest(ranges, 0.0, math.radians(1.0), pose)

    stats = builder.stats()
    print(f"\n  Ingested: {stats['scan_count']} scans, "
          f"{stats['raw_point_count']} points, "
          f"{stats['voxel_count']} occupied voxels")

    if stats["bounds"]:
        b = stats["bounds"]
        print(f"  Map bounds: X [{b['x_min_m']:.1f}, {b['x_max_m']:.1f}] m East")
        print(f"              Y [{b['y_min_m']:.1f}, {b['y_max_m']:.1f}] m North")
        print(f"              Z [{b['z_min_m']:.1f}, {b['z_max_m']:.1f}] m AGL")

    slice_data = builder.grid.get_slice_2d(alt_min=48.0, alt_max=54.0)
    print(f"\n  2D slice (48–54 m AGL): {len(slice_data)} occupied cells")

    gj = builder.geojson_slice(drone_alt=50.0)
    print(f"  GeoJSON slice features: {len(gj['features'])}")

    # Write test PCD files to /tmp
    saved = builder.save("/tmp/aran_map_test")
    print(f"\n  PCD files saved:")
    print(f"    Raw:   {saved['raw_pcd']}")
    print(f"    Voxel: {saved['voxel_pcd']}")

    print("\nSelf-test passed.")