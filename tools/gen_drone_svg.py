#!/usr/bin/env python3
"""
gen_drone_svg.py — Generate top-down + side-view SVG of mbc3_radar_drone
from SDF geometry values.

Dimensions from mbc3_radar_drone.sdf:
  Hub body:        r=0.138m, h=0.05m, z=0.1865m
  Arms (×6):       length=0.36m, r=0.008m, at 0/60/120/180/240/300 deg
  Motors (×6):     r=0.0165m at arm tips
  Prop discs (×6): estimated r=0.127m (10-in props for 3.8kg hex)
  Radar box:       0.236×0.236×0.046m, z=0.037m
  Radar panels(×6):antenna 0.015×0.132×0.092m at r≈0.181m
  Battery:         0.265×0.075×0.059m, z=0.139m
  Bottom plate:    0.22×0.22×0.003m, z=0.107m
  LG posts:        r=0.008, h=0.09m at y=±0.145m, z=-0.0625m
  LG skids:        r=0.014, l=0.22m at y=±0.145m, z=-0.1175m
"""

import math

SCALE = 330          # px per metre
W, H  = 900, 520     # SVG canvas

# Top-down view centre
TX, TY = 230, 265

# Side view centre (X-Z plane, Z up in SDF → up in SVG)
SX, SY_BASE = 660, 370   # SY_BASE = ground level z=0

def m(v): return v * SCALE   # metres → pixels (no offset)


def svg_x(world_x, cx=TX): return cx + m(world_x)
def svg_y(world_y, cy=TY): return cy - m(world_y)   # Y flipped

def side_x(world_x, cx=SX): return cx + m(world_x)
def side_y(world_z, base=SY_BASE): return base - m(world_z)

# ── Arm geometry ──────────────────────────────────────────────────────────────
ARM_ANGLES_DEG = [0, 60, 120, 180, 240, 300]
ARM_R          = 0.36      # m from hub centre to motor
HUB_R          = 0.138     # m
MOTOR_R        = 0.0165    # m
PROP_R         = 0.127     # m estimated
ARM_W          = 0.016     # m (diameter)

PANEL_R        = 0.1805    # m — antenna panel centre distance from hub centre
PANEL_H        = 0.132     # m — panel height (tangential)
PANEL_W        = 0.092     # m — panel depth (radial, visual thickness)
PANEL_ANGLES_DEG = [0, 60, 120, 180, 240, 300]   # panels A-F

RADAR_BOX      = (0.236, 0.236, 0.046)
BATTERY        = (0.265, 0.075, 0.059)
BOTTOM_PLATE   = (0.22, 0.22, 0.003)
LG_Y           = 0.145     # m
LG_SKID_LEN    = 0.22      # m
LG_SKID_Z      = -0.1175   # m
LG_SKID_R      = 0.014     # m

HUB_Z_BOT      = 0.1865 - 0.025    # 0.1615m
HUB_Z_TOP      = 0.1865 + 0.025    # 0.2115m

COLOURS = {
    "arm":       "#3a3a4a",
    "hub":       "#2d5fa0",
    "motor":     "#1a1a2a",
    "prop":      "#aaccee",
    "radar_box": "#c0392b",
    "panel":     "#e74c3c",
    "battery":   "#27ae60",
    "plate":     "#7f8c8d",
    "lg":        "#555566",
    "bg":        "#f4f6f8",
    "grid":      "#dde2ea",
    "text":      "#1a1a2a",
    "dim":       "#888",
    "fov":       "rgba(231,76,60,0.08)",
    "fov_str":   "rgba(231,76,60,0.55)",
}

lines = []

def tag(t, close=False, **kw):
    attrs = " ".join(f'{k.replace("_","-")}="{v}"' for k, v in kw.items())
    sl = "/" if close else ""
    return f"<{t} {attrs}{sl}>"

def add(*s): lines.extend(s)

# ── SVG header ────────────────────────────────────────────────────────────────
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" font-family="monospace" font-size="11">')

# Background
add(f'<rect width="{W}" height="{H}" fill="{COLOURS["bg"]}"/>')

# ── Section labels ────────────────────────────────────────────────────────────
add(f'<text x="{TX}" y="28" text-anchor="middle" font-size="13" '
    f'font-weight="bold" fill="{COLOURS["text"]}">TOP VIEW (XY plane)</text>')
add(f'<text x="{SX}" y="28" text-anchor="middle" font-size="13" '
    f'font-weight="bold" fill="{COLOURS["text"]}">SIDE VIEW (XZ plane)</text>')
add(f'<text x="{W//2}" y="{H-8}" text-anchor="middle" font-size="10" '
    f'fill="{COLOURS["dim"]}">mbc3_radar_drone  |  mass=3.83 kg  |  '
    f'motor-span=720 mm  |  6-panel FMCW radar</text>')

# ── Divider ───────────────────────────────────────────────────────────────────
add(f'<line x1="440" y1="40" x2="440" y2="{H-20}" '
    f'stroke="{COLOURS["grid"]}" stroke-width="1"/>')

# ══════════════════════════════════════════════════════════════════════════════
# TOP-DOWN VIEW
# ══════════════════════════════════════════════════════════════════════════════

# Radar FOV cones (6 panels, each ±30°)
for a_deg in PANEL_ANGLES_DEG:
    a = math.radians(a_deg)
    fov = math.radians(30)
    r_fov = m(0.45)
    x1 = svg_x(math.cos(a - fov) * 0.45)
    y1 = svg_y(math.sin(a - fov) * 0.45)
    x2 = svg_x(math.cos(a + fov) * 0.45)
    y2 = svg_y(math.sin(a + fov) * 0.45)
    cx, cy = svg_x(0), svg_y(0)
    add(f'<path d="M{cx},{cy} L{x1},{y1} A{r_fov},{r_fov} 0 0,0 {x2},{y2} Z" '
        f'fill="{COLOURS["fov"]}" stroke="{COLOURS["fov_str"]}" '
        f'stroke-width="0.8" stroke-dasharray="4,3"/>')

# Landing gear skids (top view: horizontal lines at y=±LG_Y)
for sign in (+1, -1):
    wy = sign * LG_Y
    x1 = svg_x(-LG_SKID_LEN / 2)
    x2 = svg_x(+LG_SKID_LEN / 2)
    yy = svg_y(wy)
    add(f'<line x1="{x1}" y1="{yy}" x2="{x2}" y2="{yy}" '
        f'stroke="{COLOURS["lg"]}" stroke-width="{m(LG_SKID_R*2):.1f}" '
        f'stroke-linecap="round"/>')

# Arms + props + motors
for a_deg in ARM_ANGLES_DEG:
    a = math.radians(a_deg)
    tip_x = ARM_R * math.cos(a)
    tip_y = ARM_R * math.sin(a)

    # Arm line
    add(f'<line x1="{svg_x(0)}" y1="{svg_y(0)}" '
        f'x2="{svg_x(tip_x)}" y2="{svg_y(tip_y)}" '
        f'stroke="{COLOURS["arm"]}" stroke-width="{m(ARM_W):.1f}"/>')

    # Prop disc
    add(f'<circle cx="{svg_x(tip_x)}" cy="{svg_y(tip_y)}" '
        f'r="{m(PROP_R):.1f}" fill="{COLOURS["prop"]}" '
        f'stroke="{COLOURS["arm"]}" stroke-width="1" opacity="0.7"/>')

    # Motor circle
    add(f'<circle cx="{svg_x(tip_x)}" cy="{svg_y(tip_y)}" '
        f'r="{m(MOTOR_R):.1f}" fill="{COLOURS["motor"]}"/>')

# Radar panels (6, at 60° intervals)
for a_deg in PANEL_ANGLES_DEG:
    a = math.radians(a_deg)
    px = PANEL_R * math.cos(a)
    py = PANEL_R * math.sin(a)

    # Panel rectangle (tangential — rotate PANEL_H along tangent, PANEL_W along radial)
    # In top view the panel appears as a rectangle: tangential width PANEL_H, radial depth PANEL_W
    hw = PANEL_H / 2   # half-width tangential
    hd = PANEL_W / 2   # half-depth radial

    # Four corners in world coords (tangent = perpendicular to radial)
    tang = a + math.pi / 2
    rad  = a

    corners = []
    for sw, sd in [(-1,-1),(-1,1),(1,1),(1,-1)]:
        cx = px + sw * hw * math.cos(tang) + sd * hd * math.cos(rad)
        cy = py + sw * hw * math.sin(tang) + sd * hd * math.sin(rad)
        corners.append((svg_x(cx), svg_y(cy)))

    pts = " ".join(f"{cx:.1f},{cy:.1f}" for cx, cy in corners)
    add(f'<polygon points="{pts}" fill="{COLOURS["panel"]}" '
        f'stroke="#9b1a11" stroke-width="0.8"/>')

# Hub body
add(f'<circle cx="{svg_x(0)}" cy="{svg_y(0)}" r="{m(HUB_R):.1f}" '
    f'fill="{COLOURS["hub"]}" stroke="#1a3a70" stroke-width="1.5"/>')

# Radar box (top view)
rbw = m(RADAR_BOX[0]) / 2
add(f'<rect x="{svg_x(-RADAR_BOX[0]/2):.1f}" y="{svg_y(RADAR_BOX[1]/2):.1f}" '
    f'width="{m(RADAR_BOX[0]):.1f}" height="{m(RADAR_BOX[1]):.1f}" '
    f'fill="none" stroke="{COLOURS["radar_box"]}" stroke-width="1.2" '
    f'stroke-dasharray="3,2"/>')

# Battery (top view)
add(f'<rect x="{svg_x(-BATTERY[0]/2):.1f}" y="{svg_y(BATTERY[1]/2):.1f}" '
    f'width="{m(BATTERY[0]):.1f}" height="{m(BATTERY[1]):.1f}" '
    f'fill="{COLOURS["battery"]}" opacity="0.45" stroke="#1a7040" stroke-width="0.8"/>')

# Hub centre dot
add(f'<circle cx="{svg_x(0)}" cy="{svg_y(0)}" r="3" fill="white"/>')

# Forward arrow
ax, ay = svg_x(0.16), svg_y(0)
add(f'<line x1="{svg_x(0)}" y1="{svg_y(0)}" x2="{ax}" y2="{ay}" '
    f'stroke="white" stroke-width="2" marker-end="url(#arrow)"/>')

# Dimension annotations
# Motor-to-motor span
m2m_y = svg_y(-0.42)
add(f'<line x1="{svg_x(-ARM_R)}" y1="{m2m_y}" x2="{svg_x(ARM_R)}" y2="{m2m_y}" '
    f'stroke="{COLOURS["dim"]}" stroke-width="0.8" marker-start="url(#tick)" '
    f'marker-end="url(#tick)"/>')
add(f'<text x="{svg_x(0)}" y="{m2m_y - 4}" text-anchor="middle" '
    f'fill="{COLOURS["dim"]}" font-size="10">720 mm motor-to-motor</text>')

# ══════════════════════════════════════════════════════════════════════════════
# SIDE VIEW (X-Z plane)
# ══════════════════════════════════════════════════════════════════════════════

# Ground line
add(f'<line x1="{side_x(-0.42)}" y1="{side_y(0)}" '
    f'x2="{side_x(0.42)}" y2="{side_y(0)}" '
    f'stroke="{COLOURS["grid"]}" stroke-width="1" stroke-dasharray="4,4"/>')
add(f'<text x="{side_x(0.38)}" y="{side_y(0)+12}" '
    f'fill="{COLOURS["dim"]}" font-size="9" text-anchor="end">z=0 (ground)</text>')

# Landing gear skids (side view: horizontal cylinders at z=LG_SKID_Z, y shown as x offset)
for sign in (+1, -1):
    wy = sign * LG_Y   # y position — shown as slight x offset for depth hint
    skid_y = side_y(LG_SKID_Z)
    x1 = side_x(-LG_SKID_LEN / 2)
    x2 = side_x(+LG_SKID_LEN / 2)
    add(f'<line x1="{x1}" y1="{skid_y}" x2="{x2}" y2="{skid_y}" '
        f'stroke="{COLOURS["lg"]}" stroke-width="{m(LG_SKID_R*2):.1f}" '
        f'stroke-linecap="round" opacity="0.8"/>')

# Landing gear posts (side view: vertical at x≈0, y=±0.145)
for sign in (+1, -1):
    px = sign * 0.02   # slight horizontal offset to show both posts
    top_z  = -0.018    # top of post (connects to frame bottom)
    bot_z  = -0.1175   # bottom (skid level)
    add(f'<line x1="{side_x(px)}" y1="{side_y(top_z)}" '
        f'x2="{side_x(px)}" y2="{side_y(bot_z)}" '
        f'stroke="{COLOURS["lg"]}" stroke-width="3" stroke-linecap="round"/>')

# Radar box (side: box at z=0.0375, ±0.118 x)
rb_x1 = side_x(-RADAR_BOX[0] / 2)
rb_y1 = side_y(RADAR_BOX[2])         # top face z=0.046+0.015=0.061 approx
rb_y2 = side_y(0.015)                # bottom face z=0.015
add(f'<rect x="{rb_x1:.1f}" y="{side_y(0.060):.1f}" '
    f'width="{m(RADAR_BOX[0]):.1f}" height="{m(RADAR_BOX[2]):.1f}" '
    f'fill="{COLOURS["radar_box"]}" opacity="0.7" '
    f'stroke="#9b1a11" stroke-width="1"/>')
add(f'<text x="{side_x(0)}" y="{side_y(0.060)-4}" text-anchor="middle" '
    f'fill="{COLOURS["radar_box"]}" font-size="9">RADAR</text>')

# Radar panels side view: 0° and 180° panels visible as rectangles
for sign, label in [(+1, "A"), (-1, "D")]:
    px = sign * PANEL_R
    pz = 0.0375
    add(f'<rect x="{side_x(px - PANEL_W/2):.1f}" '
        f'y="{side_y(pz + PANEL_H/2):.1f}" '
        f'width="{m(PANEL_W):.1f}" '
        f'height="{m(PANEL_H):.1f}" '
        f'fill="{COLOURS["panel"]}" stroke="#9b1a11" stroke-width="0.8"/>')
    add(f'<text x="{side_x(px):.1f}" y="{side_y(pz + PANEL_H/2) - 3}" '
        f'text-anchor="middle" fill="{COLOURS["panel"]}" font-size="9">{label}</text>')

# Bottom plate (side)
add(f'<rect x="{side_x(-BOTTOM_PLATE[0]/2):.1f}" y="{side_y(0.1085):.1f}" '
    f'width="{m(BOTTOM_PLATE[0]):.1f}" height="{m(BOTTOM_PLATE[2]):.1f}" '
    f'fill="{COLOURS["plate"]}" stroke="#555" stroke-width="0.8"/>')

# Battery (side)
add(f'<rect x="{side_x(-BATTERY[0]/2):.1f}" y="{side_y(0.139 + BATTERY[2]/2):.1f}" '
    f'width="{m(BATTERY[0]):.1f}" height="{m(BATTERY[2]):.1f}" '
    f'fill="{COLOURS["battery"]}" opacity="0.75" stroke="#1a7040" stroke-width="1"/>')
add(f'<text x="{side_x(0)}" y="{side_y(0.139):.1f}" text-anchor="middle" '
    f'fill="white" font-size="8">BAT</text>')

# Hub body cylinder (side view: rectangle)
add(f'<rect x="{side_x(-HUB_R):.1f}" y="{side_y(HUB_Z_TOP):.1f}" '
    f'width="{m(HUB_R*2):.1f}" height="{m(0.05):.1f}" '
    f'fill="{COLOURS["hub"]}" stroke="#1a3a70" stroke-width="1.5" rx="4"/>')

# Arms (side view: only 0° arm visible as horizontal line)
arm_z = 0.1865
add(f'<line x1="{side_x(-ARM_R)}" y1="{side_y(arm_z)}" '
    f'x2="{side_x(ARM_R)}" y2="{side_y(arm_z)}" '
    f'stroke="{COLOURS["arm"]}" stroke-width="{m(ARM_W):.1f}"/>')

# Motors at tips (side)
for sign in (+1, -1):
    mx = sign * ARM_R
    add(f'<circle cx="{side_x(mx)}" cy="{side_y(arm_z + MOTOR_R)}" '
        f'r="{m(MOTOR_R):.1f}" fill="{COLOURS["motor"]}"/>')
    # Prop disc (side view: thin ellipse)
    add(f'<ellipse cx="{side_x(mx)}" cy="{side_y(arm_z + MOTOR_R + 0.012)}" '
        f'rx="{m(PROP_R):.1f}" ry="3" '
        f'fill="{COLOURS["prop"]}" opacity="0.6" stroke="{COLOURS["arm"]}" stroke-width="0.8"/>')

# Height dimension annotations (side view)
def dim_h(x_world, z1, z2, label, xoffset=0.08):
    xa = side_x(x_world + xoffset)
    y1 = side_y(z1)
    y2 = side_y(z2)
    add(f'<line x1="{xa}" y1="{y1}" x2="{xa}" y2="{y2}" '
        f'stroke="{COLOURS["dim"]}" stroke-width="0.7" '
        f'marker-start="url(#tick)" marker-end="url(#tick)"/>')
    mid_y = (y1 + y2) / 2
    add(f'<text x="{xa + 4}" y="{mid_y + 4}" fill="{COLOURS["dim"]}" '
        f'font-size="9">{label}</text>')

dim_h(ARM_R,  LG_SKID_Z, 0,            "117 mm",  0.05)
dim_h(ARM_R,  0,          HUB_Z_TOP,   "211 mm",  0.05)
dim_h(-ARM_R, LG_SKID_Z, HUB_Z_TOP,   "329 mm total",  -0.22)

# ── Defs (arrowhead, tick markers) ───────────────────────────────────────────
defs = f"""<defs>
  <marker id="arrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
    <path d="M0,0 L0,6 L8,3 z" fill="white"/>
  </marker>
  <marker id="tick" markerWidth="4" markerHeight="8" refX="2" refY="4" orient="auto">
    <line x1="2" y1="0" x2="2" y2="8" stroke="{COLOURS["dim"]}" stroke-width="1"/>
  </marker>
</defs>"""
lines.insert(2, defs)

add("</svg>")

svg_out = "\n".join(lines)
out_path = "/home/boson-229/aran_mbc/images/mbc3_radar_drone.svg"
import os
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w") as f:
    f.write(svg_out)
print(f"Written: {out_path}  ({len(svg_out)} bytes)")
