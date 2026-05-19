"""
asp_fake_targets.py — Inject 5 fake radar tracks into ASP GCS for testing.

Simulates 5 airborne targets moving in circles at 10-40 m/s.
Sends to telemetry_web.py /asp_update every 0.4s.

Usage:
    python3 src/asp_fake_targets.py

Requires: requests, telemetry_web.py running on port 5000
"""
import math, time, requests, random

GCS_URL   = "http://localhost:5000/asp_update"
HOME_LAT  = 47.3977
HOME_LON  = 8.5456
DRONE_ALT = 500.0   # m AGL — MBC-3 mode

# 5 target definitions: (center_lat, center_lon, orbit_r_m, speed_ms, alt_m, phase_offset)
TARGETS = [
    {"id": "TRK-001", "clat": HOME_LAT + 0.018, "clon": HOME_LON + 0.002,
     "r": 300,  "spd": 18, "alt": 510, "phase": 0.0},
    {"id": "TRK-002", "clat": HOME_LAT - 0.022, "clon": HOME_LON + 0.015,
     "r": 500,  "spd": 35, "alt": 490, "phase": 1.2},
    {"id": "TRK-003", "clat": HOME_LAT + 0.009, "clon": HOME_LON - 0.018,
     "r": 200,  "spd": 22, "alt": 505, "phase": 2.4},
    {"id": "TRK-004", "clat": HOME_LAT - 0.005, "clon": HOME_LON + 0.025,
     "r": 400,  "spd": 40, "alt": 520, "phase": 3.7},
    {"id": "TRK-005", "clat": HOME_LAT + 0.028, "clon": HOME_LON - 0.010,
     "r": 350,  "spd": 12, "alt": 495, "phase": 0.8},
]

R_EARTH = 6371000.0
scan_count = 0

def compute_track(t, now):
    """Compute current lat/lon of target orbiting its center point."""
    period = 2 * math.pi * t["r"] / t["spd"]
    angle  = (2 * math.pi * now / period + t["phase"]) % (2 * math.pi)
    # ENU offset from center
    dx = t["r"] * math.cos(angle)
    dy = t["r"] * math.sin(angle)
    lat = t["clat"] + math.degrees(dy / R_EARTH)
    lon = t["clon"] + math.degrees(dx / (R_EARTH * math.cos(math.radians(t["clat"]))))
    # Range from HOME
    dlat = math.radians(lat - HOME_LAT) * R_EARTH
    dlon = math.radians(lon - HOME_LON) * R_EARTH * math.cos(math.radians(HOME_LAT))
    rng  = math.sqrt(dlat**2 + dlon**2)
    brg  = (math.degrees(math.atan2(dlon, dlat)) + 360) % 360
    return {
        "id":          t["id"],
        "lat":         round(lat, 6),
        "lon":         round(lon, 6),
        "range_m":     round(rng, 1),
        "bearing_deg": round(brg, 1),
        "alt_m":       t["alt"] + random.uniform(-2, 2),
        "velocity_ms": round(t["spd"] + random.uniform(-0.5, 0.5), 1),
        "confidence":  round(0.85 + random.uniform(-0.05, 0.05), 2),
        "timestamp":   now,
    }

print("ASP Fake Targets — injecting 5 tracks to http://localhost:5000")
print("Open browser: http://localhost:5000/asp")
print("Press Ctrl-C to stop\n")

while True:
    now    = time.time()
    tracks = [compute_track(t, now) for t in TARGETS]
    scan_count += 1
    payload = {
        "asp_tracks":  tracks,
        "asp_drone_id": "DRONE-L",
        "scan_count":  scan_count,
    }
    try:
        r = requests.post(GCS_URL, json=payload, timeout=0.3)
        if r.ok and scan_count % 10 == 0:
            print(f"  [{scan_count:5d}] {len(tracks)} tracks sent  "
                  f"TRK-001 @ {tracks[0]['range_m']:.0f}m brg={tracks[0]['bearing_deg']:.0f}°")
    except Exception as e:
        print(f"  GCS unreachable: {e}")
    time.sleep(0.4)
