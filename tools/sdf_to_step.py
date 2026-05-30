#!/usr/bin/env python3
"""
tools/sdf_to_step.py — Export mbc3_radar_drone as SolidWorks-compatible STEP.

Install: pip install cadquery
Run:     python3 tools/sdf_to_step.py
Output:  new_drone/mbc3_radar_drone.step   (open directly in SolidWorks)

Geometry sourced from new_drone/mbc3_radar_drone.sdf (post-audit edits).
All values in metres. SDF coordinate frame preserved.
"""

import math
import os
import sys

try:
    import cadquery as cq
    from cadquery import Location, Vector
except ImportError:
    sys.exit(
        "cadquery not found.\n"
        "Install: pip install cadquery\n"
        "  or:    conda install -c cadquery cadquery"
    )

# ── Geometry constants (from mbc3_radar_drone.sdf, post-audit) ───────────────

HUB_Z, HUB_R, HUB_L       = 0.1865, 0.138, 0.050

ARM_R, ARM_L, ARM_Z        = 0.008, 0.360, 0.1865      # arm half-span = 0.18
ARM_CENTER_R               = ARM_L / 2                  # 0.18 m from origin

MNTBOX                     = (0.048, 0.022, 0.010)      # motor-mount box
MNTBOX_R, MNTBOX_Z        = 0.198, 0.1715

MOTOR_R, MOTOR_L           = 0.0165, 0.022
MOTOR_TIP_R, MOTOR_Z       = 0.360, 0.2075

PROP_R, PROP_T             = 0.127, 0.003               # 254 mm prop (visual)

BATTERY_BOX                = (0.265, 0.075, 0.059)
BATTERY_Z                  = 0.139

FC_BOX                     = (0.236, 0.236, 0.046)
FC_Z                       = 0.0375

RADAR_CONN_BOX             = (0.060, 0.018, 0.018)
RADAR_CONN_R               = 0.140
RADAR_PANEL_BOX            = (0.015, 0.132, 0.092)
RADAR_PANEL_R              = 0.1805
RADAR_Z                    = 0.0375

LG_POST_Y, LG_POST_Z      = 0.145, -0.090
LG_POST_R, LG_POST_L      = 0.008, 0.145

LG_SKID_Y, LG_SKID_Z      = 0.145, -0.172
LG_SKID_R, LG_SKID_L      = 0.014, 0.220              # horizontal along X

STRUT_X, STRUT_Y           = 0.0675, 0.145
STRUT_Z, STRUT_PITCH       = -0.095, 0.3422            # rad
STRUT_R, STRUT_L           = 0.005, 0.1635

# ── Colour palette ────────────────────────────────────────────────────────────
C_FRAME  = cq.Color(0.18, 0.20, 0.25)
C_MOTOR  = cq.Color(0.10, 0.10, 0.10)
C_PROP   = cq.Color(0.60, 0.60, 0.60)
C_BATT   = cq.Color(0.15, 0.68, 0.38)
C_FC     = cq.Color(0.12, 0.35, 0.12)
C_RADAR  = cq.Color(0.91, 0.30, 0.24)
C_HUB    = cq.Color(0.18, 0.37, 0.63)

# ── Helpers ───────────────────────────────────────────────────────────────────

def cyl_z(r, l):
    """Cylinder centred at origin, axis = Z."""
    return cq.Workplane("XY").cylinder(l, r)


def cyl_x(r, l):
    """Cylinder centred at origin, axis = X."""
    return cq.Workplane("YZ").cylinder(l, r)


def rot_z(wp, deg):
    """Rotate Workplane shapes around world Z."""
    return wp.rotate((0, 0, 0), (0, 0, 1), deg)


def rot_y(wp, deg):
    """Rotate Workplane shapes around world Y."""
    return wp.rotate((0, 0, 0), (0, 1, 0), deg)


def tloc(x=0.0, y=0.0, z=0.0):
    return Location(Vector(x, y, z))


# ── Build assembly ────────────────────────────────────────────────────────────

assy = cq.Assembly(name="mbc3_radar_drone")

# Hub
assy.add(cyl_z(HUB_R, HUB_L), loc=tloc(0, 0, HUB_Z), name="hub", color=C_HUB)

# Arms, motor mounts, motor bells, props
for i, deg in enumerate(range(0, 360, 60)):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)

    # Arm: cylinder along X, rotated deg° around Z, centred at arc midpoint
    arm = rot_z(cyl_x(ARM_R, ARM_L), deg)
    assy.add(arm, loc=tloc(ARM_CENTER_R * ca, ARM_CENTER_R * sa, ARM_Z),
             name=f"arm_{i}", color=C_FRAME)

    # Motor mount box (at arm transition)
    mnt = rot_z(cq.Workplane("XY").box(*MNTBOX), deg)
    assy.add(mnt, loc=tloc(MNTBOX_R * ca, MNTBOX_R * sa, MNTBOX_Z),
             name=f"motor_mount_{i}", color=C_FRAME)

    # Motor bell cylinder (at arm tip)
    assy.add(cyl_z(MOTOR_R, MOTOR_L),
             loc=tloc(MOTOR_TIP_R * ca, MOTOR_TIP_R * sa, MOTOR_Z),
             name=f"motor_bell_{i}", color=C_MOTOR)

    # Propeller disc — visual sweep only
    assy.add(cyl_z(PROP_R, PROP_T),
             loc=tloc(MOTOR_TIP_R * ca, MOTOR_TIP_R * sa,
                      MOTOR_Z + MOTOR_L / 2 + PROP_T / 2),
             name=f"prop_disc_{i}", color=C_PROP)

# Battery
assy.add(cq.Workplane("XY").box(*BATTERY_BOX), loc=tloc(0, 0, BATTERY_Z),
         name="battery", color=C_BATT)

# Flight controller board
assy.add(cq.Workplane("XY").box(*FC_BOX), loc=tloc(0, 0, FC_Z),
         name="fc_board", color=C_FC)

# Radar panels — 6 × (connector box + panel box)
for i, deg in enumerate(range(0, 360, 60)):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)

    conn = rot_z(cq.Workplane("XY").box(*RADAR_CONN_BOX), deg)
    assy.add(conn, loc=tloc(RADAR_CONN_R * ca, RADAR_CONN_R * sa, RADAR_Z),
             name=f"radar_conn_{i}", color=C_RADAR)

    panel = rot_z(cq.Workplane("XY").box(*RADAR_PANEL_BOX), deg)
    assy.add(panel, loc=tloc(RADAR_PANEL_R * ca, RADAR_PANEL_R * sa, RADAR_Z),
             name=f"radar_panel_{i}", color=C_RADAR)

# Landing gear — left (+Y) and right (−Y)
for side, sy in (("L", +1), ("R", -1)):
    assy.add(cyl_z(LG_POST_R, LG_POST_L),
             loc=tloc(0, sy * LG_POST_Y, LG_POST_Z),
             name=f"lg_post_{side}", color=C_FRAME)

    assy.add(cyl_x(LG_SKID_R, LG_SKID_L),
             loc=tloc(0, sy * LG_SKID_Y, LG_SKID_Z),
             name=f"lg_skid_{side}", color=C_FRAME)

# Support struts — 4 × angled cylinders
for i, (sx, sy, pitch_sign) in enumerate([
    (+STRUT_X, +STRUT_Y, -1),
    (+STRUT_X, -STRUT_Y, -1),
    (-STRUT_X, +STRUT_Y, +1),
    (-STRUT_X, -STRUT_Y, +1),
]):
    strut = rot_y(cyl_z(STRUT_R, STRUT_L), math.degrees(pitch_sign * STRUT_PITCH))
    assy.add(strut, loc=tloc(sx, sy, STRUT_Z),
             name=f"strut_{i}", color=C_FRAME)

# ── Export ────────────────────────────────────────────────────────────────────

out = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "new_drone", "mbc3_radar_drone.step",
)
out = os.path.normpath(out)
cq.exporters.export(assy.toCompound(), out)
print(f"Saved → {out}")
print("Open in SolidWorks: File → Open → mbc3_radar_drone.step")
print("  FeatureManager will show individual parts (hub, arm_0..5, etc.)")
