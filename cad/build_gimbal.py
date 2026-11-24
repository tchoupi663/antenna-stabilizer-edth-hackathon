"""Final gimbal CAD: 2-servo leveling gimbal, axes through the antenna CoG, measured CRPA antenna.

Lineage (credit):
  * Andrii Derevianko, `gimbal/cad/build_gimbal.py` (branch andrii): the parametric approach, helpers,
    servo envelope, 1:1 parallelogram linkages, 8 mm shafts, export and render code, and the
    control architecture (two hobby servos driven by the Cube Orange through ArduPilot's servo mount
    driver, outer axis = ROLL along body X, inner axis = PITCH along body Y, joint commands
    roll = -phi, pitch = -theta, exact for ZYX Euler angles).
  * Youssef, `cad_design_c/build_design_c.py` (branch youssef-exploration): pivots moved onto the
    antenna rim so that the gimbal axes pass through the antenna CoG, frame around the antenna,
    clearance sweep, mass properties and torque budget.
  * This file: the antenna as measured by the team (photos 2026-11-21): round, diameter 370 mm,
    50 mm thick (about 10 mm fins, 33 mm metal body, 7 mm radome), mounted with 6 bolts on tabs at
    the perimeter of the back. The whole stack is re-fitted to the 370 mm antenna inside the
    400 x 400 mm envelope, and the antenna is bolted to the brackets through its 6 back tabs.
  * Components follow the baseline bill of materials of branch feat/component (components/README.md):
    standard-size DSServo DS3240 (the list's choice once the CoG is brought to the axes), 608ZZ/688ZZ
    bearings, 8 mm shafts, M3 ball-link parallelograms.

Layout (origin = intersection of the two axes = nominal antenna CoG, Z up, X forward):
  * Antenna brackets (pitch body): the antenna is held ONLY by the 6 tabs on its back (nothing
    clamps the rim). Two aluminium brackets, each a 4 mm arc plate bolted under the 3 tabs of one
    half, with an arm that rises just outside the rim (0.5 mm air gap) to the pivot at mid-thickness
    (variant "back6"). The antenna is clocked so that two tabs sit on the pitch axis (+-Y).
  * Roll frame: square frame around the antenna (x = +-188..191, y = +-196..199.5), carries the two
    608ZZ pitch bearings and the pitch servo in the +X/+Y corner; 1:1 parallelogram to the collar
    (2 x 35 mm horns + 140 mm rod; horns longer than the 25 mm of design C so the rod clears the
    bearing boss now that both sit in the same thin slab).
  * Airframe side: two posts (x = +-195..200) with 688ZZ roll bearings; the roll servo hangs under
    the +X post (axis at z = -127) and drives the frame through a 1:1 parallelogram (2 x 25 mm
    horns + 127 mm vertical rod).
  * The cable leaves the centre of the antenna back and drops through a free column.

Units: mm, degrees. Run (any Python >= 3.10):
  pip install manifold3d numpy matplotlib
  python cad/build_gimbal.py            # chosen variant, 5 deg sweep
  python cad/build_gimbal.py --fast     # 15 deg sweep (quick iteration)
  python cad/build_gimbal.py --explore  # the collar variants, 15 deg sweep each
"""

from __future__ import annotations

import itertools
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np
from manifold3d import Manifold, OpType

OUT = Path(__file__).resolve().parent
SEG = 40
G = 9.81

# ============================================================================ parameters
VARIANT = "back6"           # chosen variant: "back6" (bolted to the 6 back tabs) | "clamps" | "ring"
ENV = 200.0                 # envelope half size (400 x 400 mm, Fire Point)
EDGE = 200.0                # outermost face (0 mm margin: a frame around a 370 mm antenna needs it)

# --- real antenna (measured from the team photos; +-2 mm) -----------------------------------
ANT_R = 185.0               # metal body band radius (diameter 370 mm)
FIN_R, FIN_Z = 183.0, (-25.0, -15.0)    # finned back, ~10 mm (fins run almost to the rim)
BODY_Z = (-15.0, 18.0)                  # metal body band, ~33 mm
DOME_R, DOME_Z = 184.0, (18.0, 25.0)    # radome, ~7 mm
TAB_ANG = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)   # 6 bolt tabs on the back at the rim, 60 deg apart
#                                       # (6 bolts confirmed; equal spacing ASSUMED from the photo).
#                                       # The antenna is clocked so that two tabs sit on the pitch axis (+-Y).
TAB_R0, TAB_R1, TAB_W, TAB_Z = 165.0, 183.0, 18.0, (-25.0, -21.0)
TAB_HOLE_R = 175.0
CONN_R, CONN_Z = 12.0, (-37.0, -25.0)   # cable connector at the centre of the back
# CoG of the antenna in its own frame (origin = mid-thickness on the axis). UNKNOWN -> parameter.
ANT_COG = (0.0, 0.0, 0.0)
TRIM_Z = 8.0                # +- vertical adjustment of the clamps (slotted holes)

# --- collar -------------------------------------------------------------------------------
COL_R0, COL_R1 = 185.5, 189.0           # clamp band radial extent (0.5 mm gap + 3.5 mm wall)
COL_ARC = 30.0                          # clamp half arc around +-Y
LIP = 1.5                               # lips over the band edges (variants "clamps"/"ring" only)
BRK_T = 4.0                             # back-bracket plate thickness under the tabs (aluminium)
BRK_RIB = 8.0                           # stiffening rib under the plate, at the arm
PIV_BOSS_R = 11.0

# --- roll frame (square) --------------------------------------------------------------------
FX0, FX1 = 188.0, 191.0                 # front/back bars (carry the roll shafts), 3 mm aluminium
SY0, SY1 = 196.0, 199.5                 # side bars (carry the pitch bearings)
BOSS_Y0 = 191.5                         # pitch bearing boss inner face (collar pivot boss ends at 190.5)
FH = 12.0                               # frame half height (24 mm bars); corner radius 199.86 < 200
SHAFT_R = 4.0                           # 8 mm shafts
BRG_PITCH = (22.0, 7.0)                 # 608ZZ in the side bars (OD, width)
BRG_ROLL = (16.0, 5.0)                  # 688ZZ in the posts (OD, width)

# --- pitch drive (servo on the frame, +X/+Y corner, 1:1 parallelogram) ------------------------
P_HORN = 35.0                           # pitch horns: rod stays above the 15 mm bearing boss at +-45 deg
R_HORN = 25.0                           # roll horns
P_SV_X = 140.0                          # pitch servo output axis x (parallel to Y, at z = 0)
P_SV_Y = 189.0                          # spline top face (y); body extends to y = 152
P_HORN_Y = (189.0, 192.0)               # horn slab
P_ROD_Y = (193.0, 195.0)                # rod slab

# --- roll drive (servo on the +X post, below the swept volume, 1:1 parallelogram) -------------
R_SV_Z = -127.0                         # roll servo output axis height (parallel to X, at y = 0)
R_SV_X = 188.0                          # spline top face (x); body extends to x = 151
R_HORN_X = (188.0, 191.0)
R_ROD_X = (192.0, 194.0)

# --- airframe side ----------------------------------------------------------------------------
PX0, PX1 = 195.0, EDGE                  # posts (5 mm, 688ZZ)
POST_HY = 30.0
FOOT_Z = (-190.0, -185.0)               # feet; the airframe mounting face is at z = FOOT_Z[0]
FOOT_X0 = 150.0
FOOT_HY = 35.0
CABLE_COL_R = 25.0                      # free column kept for the cable slack loop (radius)
CABLE_COL_TOP = -100.0                  # above this the cable follows the antenna back

# Standard-size servo envelope (DSServo DS3240 ~40 x 20 x 40.5 mm, as the DS3225 drawing; Savox SV-1270TG+ 40.3 x 20.2 x 37.2):
# output axis = +Z, spline centre at (0,0,0) on the top face, body long axis = +X.
SV_X0, SV_X1 = -10.0, 30.5
SV_FX0, SV_FX1 = -17.0, 37.5
SV_HW, SV_H = 10.0, 40.5
SV_FLANGE_Z = (-11.5, -9.0)

# --- materials / bought parts -----------------------------------------------------------------
RHO = {"PETG": 1.27e-3, "Al6061": 2.70e-3}     # g/mm^3
SERVO_G = 66.0                                  # DSServo DS3240 (seller spec)
BEARING_G = {"608ZZ": 12.0, "688ZZ": 4.0}
STEEL = 7.85e-3

# --- loads (same assumptions as the design A budget) -------------------------------------------
TURN_DEG = 45.0              # coordinated turn: apparent g = g / cos 45 = 1.41 g, tilted 45 deg
PULL_UP = 0.5                # extra normal load in a pull-up (g) -> 1.5 g
RATE = math.radians(25.0)    # 25 deg/s
F_TURB = 1.2                 # Hz -> alpha = rate * 2 pi f
KC = 0.30                    # cable stiffness per joint (N.m/rad)   [assumed]
B_VISC = 0.05                # bearing viscous friction (N.m.s/rad)  [assumed]
COUL_GEAR = 0.15             # servo gear-train dry friction at the output (N.m) [assumed]
MARGIN = 2.0



# ============================================================================ helpers (Andrii)
def box(x0, x1, y0, y1, z0, z1) -> Manifold:
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(axis: str, r: float, a0: float, a1: float, c=(0.0, 0.0), seg=SEG) -> Manifold:
    """Cylinder along `axis` from a0 to a1; c = centre in the two other coords (in xyz order)."""
    m = Manifold.cylinder(a1 - a0, r, r, seg).translate([0, 0, a0])
    if axis == "z":
        return m.translate([c[0], c[1], 0])
    if axis == "x":
        return m.transform([[0, 0, 1, 0], [0, 1, 0, c[0]], [1, 0, 0, c[1]]])
    if axis == "y":
        return m.transform([[1, 0, 0, c[0]], [0, 0, 1, 0], [0, 1, 0, c[1]]])
    raise ValueError(axis)


def place(m: Manifold, ex, ey, ez, origin) -> Manifold:
    R = np.column_stack([ex, ey, ez]).astype(float)
    return m.transform(np.column_stack([R, origin]).tolist())


def union(*ms: Manifold) -> Manifold:
    ms = [m for m in ms if not m.is_empty()]
    return Manifold.batch_boolean(ms, OpType.Add) if len(ms) > 1 else ms[0]


def annulus_z(r0, r1, z0, z1, a0=None, a1=None, seg=120) -> Manifold:
    """Ring (or arc a0..a1 deg, measured from +X towards +Y) about Z."""
    ring = Manifold.cylinder(z1 - z0, r1, r1, seg) - Manifold.cylinder(z1 - z0 + 2, r0, r0, seg).translate([0, 0, -1])
    ring = ring.translate([0, 0, z0])
    if a0 is None:
        return ring
    # wedge from a0 to a1 built as a fan polygon hull
    pts = []
    big = r1 * 1.6
    for a in np.linspace(math.radians(a0), math.radians(a1), 24):
        for z in (z0 - 1, z1 + 1):
            pts.append([big * math.cos(a), big * math.sin(a), z])
    for z in (z0 - 1, z1 + 1):
        pts.append([0, 0, z])
    return ring ^ Manifold.hull_points(pts)


def servo_local(pocket: float = 0.0):
    p = pocket
    body = box(SV_X0 - p, SV_X1 + p, -SV_HW - p, SV_HW + p, -SV_H, 0 + p)
    flange = box(SV_FX0 - p, SV_FX1 + p, -SV_HW - p, SV_HW + p, *SV_FLANGE_Z)
    spline = cyl("z", 3.0 + p, 0.0, 4.0)
    return union(body, flange), spline


# pitch servo: output -> +Y, long axis -> +X, width -> Z
PITCH_SV = dict(ex=(1, 0, 0), ey=(0, 0, -1), ez=(0, 1, 0), origin=(P_SV_X, P_SV_Y, 0.0))
# roll servo: output -> +X, long axis -> -Z (hangs down), width -> Y
ROLL_SV = dict(ex=(0, 0, -1), ey=(0, 1, 0), ez=(1, 0, 0), origin=(R_SV_X, 0.0, R_SV_Z))


def placed_servo(cfg, pocket=0.0):
    body, spline = servo_local(pocket)
    return (place(body, cfg["ex"], cfg["ey"], cfg["ez"], cfg["origin"]),
            place(spline, cfg["ex"], cfg["ey"], cfg["ez"], cfg["origin"]))


# ============================================================================ parts
def part_antenna() -> Manifold:
    """Real antenna, approximate: fins + metal band + radome + 4 tabs + connector.
    Built in its own frame, then shifted so that its CoG sits on the axes (within the trim)."""
    m = union(cyl("z", FIN_R, *FIN_Z, seg=96), cyl("z", ANT_R, *BODY_Z, seg=96),
              cyl("z", DOME_R, *DOME_Z, seg=96), cyl("z", CONN_R, *CONN_Z))
    for a in TAB_ANG:
        t = box(TAB_R0, TAB_R1, -TAB_W / 2, TAB_W / 2, *TAB_Z) - cyl("z", 2.2, TAB_Z[0] - 1, TAB_Z[1] + 1, (TAB_HOLE_R, 0))
        m = m + t.rotate([0, 0, a])
    return m.translate(antenna_shift())


def antenna_shift():
    """Translation applied to the antenna: the clamps are slid on the band to put the CoG height
    on the axes (limited to +-TRIM_Z); lateral CoG offsets cannot be trimmed."""
    dz = -float(np.clip(ANT_COG[2], -TRIM_Z, TRIM_Z))
    return [0.0, 0.0, dz]


def residual_cog():
    s = antenna_shift()
    return np.array([ANT_COG[0], ANT_COG[1], ANT_COG[2] + s[2]])


def part_collar(variant: str) -> Manifold:
    z0, z1 = BODY_Z
    if variant == "ring":
        m = annulus_z(COL_R0, COL_R1 - 0.5, z0, z1)            # 3 mm full split ring
        m = m + annulus_z(ANT_R - LIP, COL_R1 - 0.5, z1, z1 + 2) + annulus_z(ANT_R - LIP, COL_R1 - 0.5, z0 - 2, z0)
        # 4 small feet to the tabs (positive location)
        for a in TAB_ANG:
            m = m + box(TAB_R0 + 4, COL_R1 - 0.5, -8, 8, TAB_Z[0] - 4, TAB_Z[0]).rotate([0, 0, a])
            m = m + box(COL_R0, COL_R1 - 0.5, -8, 8, TAB_Z[0] - 4, z0).rotate([0, 0, a])
    elif variant == "clamps":
        m = Manifold()
        for c in (90.0, -90.0):
            a0, a1 = c - COL_ARC, c + COL_ARC
            m = m + annulus_z(COL_R0, COL_R1, z0, z1, a0, a1)
            m = m + annulus_z(ANT_R - LIP, COL_R1, z1, z1 + 2, a0, a1)
            m = m + annulus_z(ANT_R - LIP, COL_R1, z0 - 2, z0, a0, a1)
    elif variant == "back6":
        # The antenna is held ONLY by its 6 mounting tabs on the back (team, 2026-11-21): nothing
        # touches the rim. Per side, one 4 mm arc plate under the 3 tabs of that half (c-60, c, c+60)
        # and an arm that rises just outside the rim (0.5 mm air gap) to the pivot at mid-thickness.
        m = Manifold()
        pz0, pz1 = TAB_Z[0] - BRK_T, TAB_Z[0]                                   # plate under the tabs
        for c in (90.0, -90.0):
            plate = annulus_z(TAB_R0 - 2, TAB_R1, pz0, pz1, c - 66.0, c + 66.0)
            # pieces built radial along +X, then turned to the pitch axis (c = +-90 deg -> +-Y)
            arm = box(COL_R0, COL_R1, -10, 10, pz0, P_HORN).rotate([0, 0, c])
            web = box(TAB_R0 - 2, COL_R1, -10, 10, pz0, pz1).rotate([0, 0, c])
            # rib under the plate (nothing may rise above the plate inside the rim: fins live there)
            rib = Manifold.hull_points([[x, t, z] for t in (-2.0, 2.0) for (x, z) in
                                        ((TAB_R0 - 2, pz0), (COL_R1, pz0), (COL_R1, pz0 - BRK_RIB))]).rotate([0, 0, c])
            br = plate + arm + web + rib
            for a in (c - 60.0, c, c + 60.0):                                    # M4 through each tab
                br = br - cyl("z", 2.2, pz0 - 1, pz1 + 1, (TAB_HOLE_R, 0)).rotate([0, 0, a])
            m = m + br
    else:
        raise ValueError(variant)
    # pivot bosses at +-Y (mid body height) and the pitch-horn boss on +Y (pin at z = +P_HORN)
    m = m + cyl("y", PIV_BOSS_R, COL_R0, COL_R1 + 1.5) + cyl("y", PIV_BOSS_R, -COL_R1 - 1.5, -COL_R0)
    # pitch-horn boss on +Y: pin at (x=0, z=+P_HORN), above the band, outside the radome radius
    m = m + cyl("y", 6.5, COL_R0, P_HORN_Y[1], (0, P_HORN)) + box(-6.5, 6.5, COL_R0, COL_R1, 0, P_HORN)
    m = m - cyl("y", 1.6, 170, 200, (0, P_HORN))                  # M3 pin hole
    return m


def part_pitch_shafts() -> Manifold:
    return cyl("y", SHAFT_R, COL_R1, SY1) + cyl("y", SHAFT_R, -SY1, -COL_R1)


def part_pitch_pin() -> Manifold:
    return cyl("y", 1.5, P_HORN_Y[0], P_ROD_Y[1], (0, P_HORN))


def part_frame() -> Manifold:
    m = Manifold()
    for s in (1, -1):
        xb = (FX0, FX1) if s > 0 else (-FX1, -FX0)
        yb = (SY0, SY1) if s > 0 else (-SY1, -SY0)
        m = m + box(*xb, -SY1, SY1, -FH, FH)                         # front / back bar
        m = m + box(-FX1, FX1, *yb, -FH, FH)                         # side bar
        m = m + cyl("x", 12.0, *xb)                                   # roll shaft boss
        yp = (BOSS_Y0, SY1) if s > 0 else (-SY1, -BOSS_Y0)
        m = m + cyl("y", 15.0, *yp)                                   # pitch bearing boss
        m = m - cyl("y", BRG_PITCH[0] / 2, yp[0] - 1, yp[1] + 1)     # 608ZZ seat
        m = m - cyl("x", SHAFT_R, xb[0] - 1, xb[1] + 1)               # shaft bore (pressed shaft)
    # roll-horn boss on the +X bar: pin at (y = +R_HORN, z = 0)
    m = m - cyl("x", 1.6, FX0 - 1, FX1 + 1, (R_HORN, 0))
    # lightening windows in the side bars (keep 20 mm webs around bosses and corners)
    for s in (1, -1):
        for x0 in (-172.0, -136.0, -100.0, -64.0, 28.0, 64.0, 100.0, 136.0):
            if s > 0 and x0 >= 120:     # keep the +X/+Y corner solid for the servo bracket
                continue
            yb = (SY0 - 1, SY1 + 1) if s > 0 else (-SY1 - 1, -SY0 + 1)
            m = m - box(x0, x0 + 30, *yb, -FH + 5, FH - 5)
        xb = (FX0 - 1, FX1 + 1) if s > 0 else (-FX1 - 1, -FX0 + 1)
        for y0 in (-180.0, -144.0, -108.0, -72.0, 44.0, 80.0, 116.0, 152.0):
            m = m - box(*xb, y0, y0 + 28, -FH + 4, FH - 4)
    # pitch-servo corner bracket (+X/+Y): from the side and front bars to the servo flange
    sv_fl_y1 = P_SV_Y + SV_FLANGE_Z[1]                                # flange face (y = 172.5)
    br = box(163.0, FX0, sv_fl_y1, SY0, -FH, FH)
    br = br + box(118.0, FX0, sv_fl_y1 - 7.0, sv_fl_y1 + 1.0, -FH, -SV_HW - 0.5)  # under-shelf
    sv, _ = placed_servo(PITCH_SV, pocket=0.3)
    br = br - sv
    for x in (P_SV_X - 13.5, P_SV_X + 34.0):
        for dz in (-5.0, 5.0):
            br = br - cyl("y", 1.6, 160, 190, (x, dz))
    return m + br


def part_roll_shafts() -> Manifold:
    return cyl("x", SHAFT_R, FX0, PX1) + cyl("x", SHAFT_R, -PX1, -FX0)


def part_roll_pin() -> Manifold:
    return cyl("x", 1.5, FX1 - 1, R_ROD_X[1], (R_HORN, 0))


def part_post(side: int) -> Manifold:
    x0, x1 = (PX0, PX1) if side > 0 else (-PX1, -PX0)
    m = box(x0, x1, -POST_HY, POST_HY, FOOT_Z[1], 0) + cyl("x", 16.0, x0, x1)
    m = m - cyl("x", BRG_ROLL[0] / 2, x0 - 1, x1 + 1)                  # 688ZZ seat
    m = m - box(x0 - 1, x1 + 1, -POST_HY + 7, POST_HY - 7, -165, -26)  # lightening window
    fx = (FOOT_X0, EDGE) if side > 0 else (-EDGE, -FOOT_X0)
    m = m + box(*fx, -FOOT_HY, FOOT_HY, *FOOT_Z)                         # foot on the airframe
    for xx, yy in itertools.product((FOOT_X0 + 12, EDGE - 10), (-25, 25)):
        m = m - cyl("z", 2.2, FOOT_Z[0] - 1, FOOT_Z[1] + 1, (side * xx, yy))   # M4
    # gusset plate on the -Y edge (triangle in XZ), stiffens the post fore-aft
    xi = FOOT_X0 if side > 0 else -FOOT_X0
    xo = x0 if side > 0 else x1
    pts = [[xo, y, z] for y in (-FOOT_HY, -FOOT_HY + 6) for z in (FOOT_Z[1], -100.0)]
    pts += [[xi, y, FOOT_Z[1]] for y in (-FOOT_HY, -FOOT_HY + 6)]
    m = m + Manifold.hull_points(pts)
    if side > 0:                                                         # roll-servo mount plate
        plate = box(R_SV_X + SV_FLANGE_Z[0] - 6.0, R_SV_X + SV_FLANGE_Z[0], -FOOT_HY, 22.0, FOOT_Z[1], -100.0)
        sv, _ = placed_servo(ROLL_SV, pocket=0.3)
        plate = plate - sv
        for zz in (R_SV_Z + 13.5, R_SV_Z - 34.0):
            for dy in (-5.0, 5.0):
                plate = plate - cyl("x", 1.6, 160, 190, (dy, zz))
        m = m + plate
    return m


def part_horn_x(x0, x1, hub=(0.0, 0.0), angle=0.0) -> Manifold:
    """Horn in a YZ slab, hub at (y,z)=hub, arm towards +Y, rotated by `angle` about X."""
    h = box(x0, x1, -5, R_HORN, -4, 4) + cyl("x", 7, x0, x1) + cyl("x", 5, x0, x1, (R_HORN, 0))
    h = h - cyl("x", 1.6, x0 - 1, x1 + 1, (R_HORN, 0))
    return h.rotate([angle, 0, 0]).translate([0, hub[0], hub[1]])


def part_horn_y(y0, y1, hub_x=0.0, angle=0.0) -> Manifold:
    """Horn in an XZ slab, hub at (x=hub_x, z=0), arm towards +Z, rotated by `angle` about Y."""
    h = box(-4, 4, y0, y1, -5, P_HORN) + cyl("y", 7, y0, y1) + cyl("y", 5, y0, y1, (0, P_HORN))
    h = h - cyl("y", 1.6, y0 - 1, y1 + 1, (0, P_HORN))
    return h.rotate([0, angle, 0]).translate([hub_x, 0, 0])


def part_roll_rod(roll: float) -> Manifold:
    a = math.radians(roll)
    y, z = R_HORN * math.cos(a), R_HORN * math.sin(a)
    x0, x1 = R_ROD_X
    return (box(x0, x1, y - 3, y + 3, R_SV_Z + z, z)
            + cyl("x", 5, x0, x1, (y, z)) + cyl("x", 5, x0, x1, (y, R_SV_Z + z)))


def part_pitch_rod(pitch: float) -> Manifold:
    """In frame coordinates. Horn tip (0, +P_HORN) rotated by pitch about Y."""
    a = math.radians(pitch)
    x, z = P_HORN * math.sin(a), P_HORN * math.cos(a)
    y0, y1 = P_ROD_Y
    return (box(x, x + P_SV_X, y0, y1, z - 3, z + 3)
            + cyl("y", 5, y0, y1, (x, z)) + cyl("y", 5, y0, y1, (x + P_SV_X, z)))


def cable_column() -> Manifold:
    """Keep-out column for the cable slack loop: antenna back -> airframe."""
    return cyl("z", CABLE_COL_R, FOOT_Z[1], CABLE_COL_TOP)


def cable_dummy() -> Manifold:
    """Visual only: 12 mm braided cable with an S-shaped slack loop in the free column."""
    pts = [(0, 0, -37), (0, 0, -55), (14, 0, -75), (14, 0, -95), (-14, 0, -115), (-14, 0, -135),
           (0, 0, -155), (0, 0, -186)]
    segs = [Manifold.hull(Manifold.sphere(6, 16).translate(a) + Manifold.sphere(6, 16).translate(b))
            for a, b in zip(pts[:-1], pts[1:])]
    return union(*segs)


# ============================================================================ assembly
def build(variant: str):
    S = {}
    S["base"] = union(part_post(+1), part_post(-1))
    S["roll_servo"] = union(*placed_servo(ROLL_SV))
    S["frame"] = part_frame()
    S["pitch_servo"] = union(*placed_servo(PITCH_SV))
    S["collar"] = part_collar(variant)
    S["antenna"] = part_antenna()
    S["roll_shafts"] = part_roll_shafts()
    S["pitch_shafts"] = part_pitch_shafts()
    S["roll_pin"] = part_roll_pin()
    S["pitch_pin"] = part_pitch_pin()
    return S


def posed(S, roll: float, pitch: float) -> dict:
    """Gimbal joint angles (deg). To level: roll = -airframe_roll, pitch = -airframe_pitch."""
    R = lambda m: m.rotate([roll, 0, 0])                                  # noqa: E731
    RP = lambda m: m.rotate([0, pitch, 0]).rotate([roll, 0, 0])           # noqa: E731
    r_horn = part_horn_x(*R_HORN_X, hub=(0.0, R_SV_Z), angle=roll)
    p_horn = part_horn_y(*P_HORN_Y, hub_x=P_SV_X, angle=pitch)
    return {
        "base": (S["base"], "base"),
        "roll_servo": (S["roll_servo"], "servo"),
        "roll_servo_horn": (r_horn, "link"),
        "roll_rod": (part_roll_rod(roll), "link"),
        "frame": (R(S["frame"]), "ring"),
        "roll_shafts": (R(S["roll_shafts"]), "shaft"),
        "roll_pin": (R(S["roll_pin"]), "shaft"),
        "pitch_servo": (R(S["pitch_servo"]), "servo"),
        "pitch_servo_horn": (R(p_horn), "link"),
        "pitch_rod": (R(part_pitch_rod(pitch)), "link"),
        "collar": (RP(S["collar"]), "cradle"),
        "pitch_shafts": (RP(S["pitch_shafts"]), "shaft"),
        "pitch_pin": (RP(S["pitch_pin"]), "shaft"),
        "antenna": (RP(S["antenna"]), "antenna"),
    }


GROUP = {"base": "A", "roll_servo": "A", "roll_servo_horn": "RH", "roll_rod": "RR",
         "frame": "B", "pitch_servo": "B", "roll_pin": "B", "roll_shafts": "RS",
         "pitch_servo_horn": "PH", "pitch_rod": "PR", "pitch_shafts": "PS",
         "collar": "C", "antenna": "C", "pitch_pin": "C"}
JOINED = {frozenset(p) for p in [
    ("roll_servo_horn", "roll_servo"), ("roll_servo_horn", "roll_rod"), ("roll_rod", "roll_pin"),
    ("roll_shafts", "base"), ("roll_shafts", "frame"),
    ("pitch_servo_horn", "pitch_servo"), ("pitch_servo_horn", "pitch_rod"), ("pitch_rod", "pitch_pin"),
    ("pitch_shafts", "frame"), ("pitch_shafts", "collar")]}
MOVING = ["roll_servo_horn", "roll_rod", "frame", "pitch_servo", "pitch_servo_horn", "pitch_rod",
          "collar", "antenna", "roll_shafts", "pitch_shafts"]


def _bbox_gap(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    d = [max(bb[i] - ba[i + 3], ba[i] - bb[i + 3], 0.0) for i in range(3)]
    return math.sqrt(sum(x * x for x in d))


def check(S, step=5.0, limit=(50.0, 45.0), search=12.0):
    """Sweep roll -limit[0]..+limit[0] x pitch -limit[1]..+limit[1] (joint travel of the model)."""
    r_ang = np.unique(np.r_[np.arange(-limit[0], limit[0] + 1e-9, step), -limit[0], limit[0]])
    p_ang = np.unique(np.r_[np.arange(-limit[1], limit[1] + 1e-9, step), -limit[1], limit[1]])
    names = list(GROUP)
    worst, env, ant_z, col = {}, [0.0, 0.0], 1e9, {}
    column = cable_column()  # noqa
    for r, p in itertools.product(r_ang, p_ang):
        parts = posed(S, r, p)
        for a, b in itertools.combinations(names, 2):
            if GROUP[a] == GROUP[b] or frozenset((a, b)) in JOINED:
                continue
            ma, mb = parts[a][0], parts[b][0]
            if _bbox_gap(ma, mb) >= search:
                gap = search
            else:
                inter = ma ^ mb
                gap = -inter.volume() if inter.volume() > 1e-3 else ma.min_gap(mb, search)
            if (a, b) not in worst or gap < worst[(a, b)][0]:
                worst[(a, b)] = (gap, r, p)
        for n in MOVING:                                   # cable column must stay free
            m = parts[n][0]
            g = search if _bbox_gap(m, column) >= search else (
                -(m ^ column).volume() if (m ^ column).volume() > 1e-3 else m.min_gap(column, search))
            if n not in col or g < col[n][0]:
                col[n] = (g, r, p)
        for n, (m, _) in parts.items():                   # envelope (everything but the antenna)
            if n == "antenna":
                continue
            bb = m.bounding_box()
            env[0] = max(env[0], abs(bb[0]), abs(bb[3]))
            env[1] = max(env[1], abs(bb[1]), abs(bb[4]))
        ant_z = min(ant_z, parts["antenna"][0].bounding_box()[2])
    return worst, env, ant_z, col


# ============================================================================ mass properties
def mass_props(m: Manifold, rho: float):
    """Exact mass (g), CoG (mm) and inertia tensor about the origin (kg.m^2) of a closed mesh."""
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype(float)
    t = np.asarray(mesh.tri_verts)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    V = vol.sum()
    com = ((a + b + c) / 4.0 * vol[:, None]).sum(0) / V
    s = a + b + c
    C = (np.einsum("n,ni,nj->ij", vol, a, a) + np.einsum("n,ni,nj->ij", vol, b, b)
         + np.einsum("n,ni,nj->ij", vol, c, c) + np.einsum("n,ni,nj->ij", vol, s, s)) / 20.0
    I = (np.trace(C) * np.eye(3) - C) * rho * 1e-3 * 1e-6           # g.mm^2 -> kg.m^2
    return V * rho, com, I


def point_mass_inertia(mass_g, p):
    p = np.asarray(p, float)
    return mass_g * 1e-3 * 1e-6 * (p @ p * np.eye(3) - np.outer(p, p))


# ============================================================================ torque budget
def alpha():
    return RATE * 2 * math.pi * F_TURB


def load_factor():
    return max(1.0 / math.cos(math.radians(TURN_DEG)), 1.0 + PULL_UP)


def axis_torque(m_kg, offset_m, J):
    """Worst-case torque (N.m) on one axis: apparent gravity on the CoG offset (any direction,
    1.5 g worst of the 45 deg turn / pull-up), inertia at 25 deg/s 1.2 Hz, cable at 45 deg,
    viscous bearing friction, servo gear friction."""
    t = {"cog": m_kg * G * load_factor() * offset_m,
         "inertia": J * alpha(),
         "cable": KC * math.radians(45.0),
         "viscous": B_VISC * RATE,
         "gear_friction": COUL_GEAR}
    t["total"] = sum(t.values())
    t["required_x2"] = MARGIN * t["total"]
    return t


def kgcm(nm):
    return nm / 0.0980665


# ============================================================================ export (Andrii)
def write_stl(m: Manifold, path: Path):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype(np.float32)
    t = np.asarray(mesh.tri_verts)
    tri = v[t]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    with open(path, "wb") as f:
        f.write(path.stem.encode()[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(t)))
        rec = np.zeros(len(t), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"], rec["v"] = n, tri
        f.write(rec.tobytes())


COLORS = {"base": "#6b7280", "servo": "#1f2937", "link": "#d97706", "ring": "#2563eb",
          "shaft": "#9ca3af", "cradle": "#059669", "antenna": "#e5e7eb", "cable": "#7c3aed"}


def render(parts, path: Path, title: str, show_antenna=True, view=(22, -55)):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(projection="3d")
    for name, (m, kind) in parts.items():
        if kind == "antenna" and not show_antenna:
            continue
        mesh = m.to_mesh()
        v = np.asarray(mesh.vert_properties)[:, :3]
        tri = v[np.asarray(mesh.tri_verts)]
        alpha_ = 0.22 if kind == "antenna" else 1.0
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.55 + 0.45 * np.clip(n @ np.array([0.3, -0.5, 0.8]), 0, 1)
        base = np.array(matplotlib.colors.to_rgb(COLORS[kind]))
        pc = Poly3DCollection(tri, edgecolor="none")
        pc.set_facecolor(np.column_stack([np.outer(shade, base).clip(0, 1), np.full(len(tri), alpha_)]))
        ax.add_collection3d(pc)
    # 400 x 400 envelope outline at the mounting face
    zf = FOOT_Z[0]
    ex = [-ENV, ENV, ENV, -ENV, -ENV]
    ey = [-ENV, -ENV, ENV, ENV, -ENV]
    ax.plot(ex, ey, [zf] * 5, color="#dc2626", lw=1.0, ls="--")
    ax.set_xlim(-210, 210); ax.set_ylim(-210, 210); ax.set_zlim(-230, 190)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X / roll axis [mm]"); ax.set_ylabel("Y / pitch axis [mm]"); ax.set_zlabel("Z [mm]")
    ax.view_init(elev=view[0], azim=view[1])
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ============================================================================ reports
def structure_parts(S):
    """Printed / machined structure (no servos, shafts, antenna)."""
    return {"posts_feet_x2": S["base"], "roll_frame": S["frame"], "collar": S["collar"]}


def summarize(variant, S, worst, env, ant_z, col):
    st = structure_parts(S)
    vol = {k: m.volume() / 1000 for k, m in st.items()}
    tot = sum(vol.values())
    clear = min(g for g, _, _ in worst.values())
    pair = min(worst, key=lambda k: worst[k][0])
    return {"variant": variant, "volume_cm3": vol, "volume_total_cm3": tot,
            "mass_PETG_g": tot * RHO["PETG"] * 1000, "mass_Al_g": tot * RHO["Al6061"] * 1000,
            "min_clearance_mm": clear, "min_pair": f"{pair[0]} vs {pair[1]} @ r{worst[pair][1]:+.0f} p{worst[pair][2]:+.0f}",
            "envelope_half_xy_mm": env, "antenna_min_z_mm": ant_z,
            "antenna_to_mount_face_mm": ant_z - FOOT_Z[0], "antenna_to_feet_top_mm": ant_z - FOOT_Z[1],
            "cable_column_min_gap_mm": min(g for g, _, _ in col.values())}


def explore():
    rows = []
    for v in ("back6", "clamps", "ring"):
        S = build(v)
        worst, env, ant_z, col = check(S, step=15.0)
        rows.append(summarize(v, S, worst, env, ant_z, col))
        print(json.dumps(rows[-1], indent=1, default=float))
        print("   pairs <3 mm:", {f"{a}/{b}": round(g, 1) for (a, b), (g, r, p) in worst.items() if g < 3})
        print("   column:", {k: round(v[0], 1) for k, v in col.items() if v[0] < 12})
    (OUT / "variants_explore.json").write_text(json.dumps(rows, indent=1, default=float))


def main():
    if "--explore" in sys.argv:
        explore()
        return
    step = 15.0 if "--fast" in sys.argv else 5.0
    S = build(VARIANT)

    # ---------------------------------------------------------------- STLs
    stl = OUT / "stl"
    stl.mkdir(exist_ok=True)
    printable = {
        "post_servo_side": part_post(+1),
        "post_bearing_side": part_post(-1),
        "roll_frame": S["frame"],
        "antenna_brackets": S["collar"],
        "horn_x4": part_horn_x(*R_HORN_X),
        "roll_rod": part_roll_rod(0.0),
        "pitch_rod": part_pitch_rod(0.0),
        "servo_dummy": union(*servo_local()),
        "antenna_crpa_d370": part_antenna(),
        "cable_dummy": cable_dummy(),
    }
    for name, m in printable.items():
        write_stl(m, stl / f"{name}.stl")
    # visual parts for the Simulink 3D view, in their HOME pose and assembly coordinates (origin = axis
    # crossing): the model moves them with the joints (horns turn, parallelogram rods translate)
    home = posed(S, 0, 0)
    for key in ("roll_servo_horn", "roll_rod", "roll_shafts", "roll_pin",
                "pitch_servo_horn", "pitch_rod", "pitch_shafts", "pitch_pin"):
        write_stl(home[key][0], stl / f"vis_{key}.stl")
    poses = {"home": (0, 0), "airframe_r30_p-20": (-30, 20), "airframe_r45_p45": (-45, -45)}
    for tag, (r, p) in poses.items():
        parts = posed(S, r, p)
        if tag == "home":
            parts["cable"] = (cable_dummy(), "cable")
        write_stl(union(*[m for m, _ in parts.values()]), stl / f"assembly_{tag}.stl")
        render(parts, OUT / f"render_{tag}.png", f"Final gimbal  joints roll={r:+.0f}, pitch={p:+.0f} deg")
    hp = posed(S, 0, 0)
    hp["cable"] = (cable_dummy(), "cable")
    render(hp, OUT / "render_mechanism.png", "Final gimbal mechanism (antenna hidden)", show_antenna=False)
    render(posed(S, 0, 0), OUT / "render_front.png", "Final gimbal home, view along -X", view=(5, 0))

    # ---------------------------------------------------------------- checks
    worst, env, ant_z, col = check(S, step=step)
    summ = summarize(VARIANT, S, worst, env, ant_z, col)
    print(f"variant {VARIANT}: clearance sweep roll +-50 x pitch +-45 deg in {step:.0f} deg steps (worst per pair)")
    bad = False
    for (a, b), (gap, r, p) in sorted(worst.items(), key=lambda kv: kv[1][0])[:14]:
        flag = f"COLLISION ({-gap:.1f} mm3)" if gap <= 0 else ("tight" if gap < 2 else "")
        bad |= gap <= 0
        print(f"  {a:>16} vs {b:<16} min gap {max(gap, 0):5.1f} mm @ roll {r:+.0f} pitch {p:+.0f} {flag}")
    print("  cable column:", {k: (round(v[0], 1), v[1], v[2]) for k, v in col.items() if v[0] < 12})
    print(f"  envelope half sizes (no antenna): x {env[0]:.1f}  y {env[1]:.1f}  (limit {ENV:.0f})")
    print(f"  antenna lowest point {ant_z:.1f} mm; feet top {FOOT_Z[1]}; mounting face {FOOT_Z[0]}")
    print("RESULT:", "collisions found" if bad else "no collisions across roll +-50 x pitch +-45 deg")

    # ---------------------------------------------------------------- mass properties
    rho_main = RHO["Al6061"]
    m_col, c_col, I_col = mass_props(S["collar"], rho_main)
    m_fr, c_fr, I_fr = mass_props(S["frame"], rho_main)
    sv_body = union(*placed_servo(PITCH_SV))
    _, c_sv, _ = mass_props(sv_body, 1.0)
    I_sv = point_mass_inertia(SERVO_G, c_sv)
    shaft_g = (math.pi * SHAFT_R ** 2) * ((SY1 + 1 - COL_R1) * 2) * STEEL
    I_psh = point_mass_inertia(shaft_g, (0, 190, 0)) * 1.0
    # pitch bearings + horn + rod + pins lumped at the +Y bearing / servo
    lump_B = 2 * BEARING_G["608ZZ"] + 15.0
    I_lump = point_mass_inertia(2 * BEARING_G["608ZZ"], (0, 195, 0)) + point_mass_inertia(15.0, (60, 187, 0))

    m_rf = m_fr + SERVO_G + lump_B                              # roll body without the pitch body (g)
    com_rf = (m_fr * c_fr + SERVO_G * c_sv + 2 * BEARING_G["608ZZ"] * np.array([0, 0, 0])
              + 15.0 * np.array([60, 187, 0])) / m_rf
    I_rf = I_fr + I_sv + I_lump

    out = {"variant": VARIANT, "summary": summ}
    budget = {}
    for m_ant in (2.0, 4.0):
        # antenna: uniform disc R=180, t=50 about its own CoG
        R, t = ANT_R / 1000, 0.050
        Jd = m_ant * (R ** 2 / 4 + t ** 2 / 12)
        Jz = m_ant * R ** 2 / 2
        e = residual_cog() / 1000
        I_ant = np.diag([Jd, Jd, Jz]) + m_ant * (e @ e * np.eye(3) - np.outer(e, e))
        I_pitch_body = I_ant + I_col + I_psh
        J_pitch = I_pitch_body[1, 1]
        J_roll = I_pitch_body[0, 0] + I_rf[0, 0]
        m_pb = m_ant + (m_col + shaft_g) / 1000
        res = {}
        for e_mm in (0.0, 5.0, 10.0, 20.0):
            off_p = e_mm / 1000 + np.linalg.norm(c_col[[0, 2]]) / 1000 * (m_col / 1000) / m_pb
            tp = axis_torque(m_pb, off_p, J_pitch)
            # roll: payload offset + roll-frame imbalance (pitch servo in the corner)
            imb = (m_rf / 1000) * G * load_factor() * math.hypot(com_rf[1], com_rf[2]) / 1000
            tr = axis_torque(m_pb, e_mm / 1000, J_roll)
            tr["frame_imbalance"] = imb
            tr["total"] += imb
            tr["required_x2"] = MARGIN * tr["total"]
            res[f"e{e_mm:.0f}mm"] = {"pitch": tp, "roll": tr}
        budget[f"{m_ant:.0f}kg"] = {"J_pitch_kgm2": J_pitch, "J_roll_kgm2": J_roll,
                                   "pitch_body_mass_kg": m_pb, "cases": res}
    out["budget"] = budget
    out["roll_frame"] = {"mass_g_Al": m_rf, "com_mm": com_rf.tolist(),
                         "J_about_origin_kgm2": np.diag(I_rf).tolist()}
    out["collar"] = {"mass_g_Al": m_col, "com_mm": c_col.tolist()}

    # servo selection (baseline bill of materials, branch feat/component): DSServo DS3240, 180 deg version.
    # Seller spec: 36 / 45 kg.cm and 0.20 / 0.17 s per 60 deg at 5.0 / 6.8 V, 4.8-6.8 V. Run from the
    # Hobbywing UBEC at 6.0 V. Buyer reviews report less torque than rated -> DERATED to 70 %.
    SERVO = {"model": "DSServo DS3240 (180 deg)", "stall_kgcm_seller": {"5.0V": 36, "6.8V": 45},
             "speed_s_per_60deg": {"5.0V": 0.20, "6.8V": 0.17}, "supply_V": 6.0,
             "stall_kgcm_used_6V_derated": 28.0, "speed_s_per_60deg_used_6V": 0.185,
             "mass_g": SERVO_G, "price_eur": "~20"}
    t_srv = SERVO["stall_kgcm_used_6V_derated"] * 0.0980665
    emax = {}
    for mk, B in budget.items():
        c0 = B["cases"]["e0mm"]
        m_pb = B["pitch_body_mass_kg"]
        for ax in ("pitch", "roll"):
            fixed = c0[ax]["total"] - c0[ax]["cog"]
            emax[f"{mk}_{ax}"] = 1000 * (t_srv / MARGIN - fixed) / (m_pb * G * load_factor())
    out["servo"] = SERVO
    out["max_residual_cog_offset_mm_for_servo_x2"] = emax

    # design A with the same budget method (Andrii: h = 93 mm, square plate 400 mm, frame J1 0.008)
    A = {}
    for m_ant in (2.0, 4.0):
        J2 = m_ant * 0.4 ** 2 / 12 + m_ant * 0.093 ** 2
        # vertical offset: only the horizontal part of the apparent g acts (g tan 45 = 1 g)
        tp = axis_torque(m_ant, 0.093 / load_factor(), J2)
        tr = axis_torque(m_ant, 0.093 / load_factor(), J2 + 0.008)
        andrii = m_ant * G * 0.093 + J2 * alpha()
        tp["andrii_method_turn_plus_inertia"] = andrii
        A[f"{m_ant:.0f}kg"] = {"pitch": tp, "roll": tr, "J": J2}
    out["design_A_same_method"] = A
    (OUT / "gimbal_results.json").write_text(json.dumps(out, indent=1, default=float))

    # ---------------------------------------------------------------- print the key numbers
    print(json.dumps(summ, indent=1, default=float))
    for mk, B in budget.items():
        for ek, c in B["cases"].items():
            print(f"{mk} {ek}: pitch {c['pitch']['total']:.2f} N.m (x2 {kgcm(c['pitch']['required_x2']):.1f} kg.cm)"
                  f" | roll {c['roll']['total']:.2f} N.m (x2 {kgcm(c['roll']['required_x2']):.1f} kg.cm)")
        print(f"   J_pitch {B['J_pitch_kgm2']:.4f}  J_roll {B['J_roll_kgm2']:.4f} kg.m2")
    print("max residual offset for the DS3240 (6 V, derated) with x2:", {k: round(v, 1) for k, v in emax.items()})
    for mk, a in A.items():
        print(f"design A {mk}: roll {a['roll']['total']:.2f} N.m (x2 {kgcm(a['roll']['required_x2']):.0f} kg.cm), "
              f"Andrii's method {a['pitch']['andrii_method_turn_plus_inertia']:.2f} N.m")
    print("roll frame", out["roll_frame"], "collar", out["collar"])

    # ---------------------------------------------------------------- simulation parameters
    write_params(out, SERVO)


def write_params(out, SERVO):
    B4, B2 = out["budget"]["4kg"], out["budget"]["2kg"]
    rf = out["roll_frame"]
    R, t = ANT_R / 1000, 0.050
    L_eq = math.sqrt(12 * (R ** 2 / 4 + t ** 2 / 12))
    params = {
        "_doc": ("Final gimbal (servo gimbal, axes through the CoG of the measured 370 x 50 mm antenna): "
                 "values derived from the CAD for the MATLAB/Simulink model (sim/). "
                 "Generated by build_gimbal.py."),
        "variant": out["variant"],
        "geometry": {
            "cog_offset_nominal_m": residual_cog().tolist(),
            "h_m": 0.005, "h_note": "residual CoG offset assumed for the sim (+ = above the axes); "
                                     "nominal geometry puts the CoG on the axes. Sweep 0..0.020 m.",
            "L_equivalent_m": L_eq,
            "L_note": "side of the square plate with the same inertia about a diameter as the "
                      "370 x 50 mm disc",
            "antenna_radius_m": R, "antenna_thickness_m": t,
        },
        "mass_options_kg": [2.0, 4.0],
        "inertia_kgm2": {
            "4kg": {"J_pitch": B4["J_pitch_kgm2"], "J_roll": B4["J_roll_kgm2"]},
            "2kg": {"J_pitch": B2["J_pitch_kgm2"], "J_roll": B2["J_roll_kgm2"]},
            "note": "uniform disc; a rim-heavy antenna (metal band) can be up to ~2x on the antenna term",
        },
        "m1": rf["mass_g_Al"] / 1000, "J1": rf["J_about_origin_kgm2"],
        "m1_note": "roll frame + pitch servo + pitch bearings/linkage, aluminium frame",
        "frame_imbalance_roll_m": math.hypot(rf["com_mm"][1], rf["com_mm"][2]) / 1000,
        "servo": {
            "model": SERVO["model"], "supply_V": SERVO["supply_V"],
            "tauStall": SERVO["stall_kgcm_used_6V_derated"] * 0.0980665,
            "w0": math.radians(60 / SERVO["speed_s_per_60deg_used_6V"]),
            "w0_note": "no-load speed (rad/s) at 6.0 V, interpolated from the seller spec",
            "deadband": math.radians(0.1), "thetaPB": math.radians(2.0), "zeta": 0.7,
            "Jm": 0.01, "kg": 500, "zg": 0.2, "coulM": COUL_GEAR,
            "backlash": [math.radians(0.5), math.radians(0.3)],
            "backlash_note": "roll = servo + 1:1 linkage (2 ball links), pitch now ALSO through a "
                             "1:1 linkage -> consider 0.5 deg on both axes",
        },
        "delays_same_as_design_A": {
            "delayPure": 0.010, "mountRate": 400, "pwmRate": 333, "ff": False, "tauFF": 0.045,
            "cubeNoise_deg": 0.05,
            "note": ("Andrii's assumptions (400 Hz attitude, 333 Hz servo frame, ~40-50 ms total). "
                     "Market-benchmark check: ArduPilot's servo mount backend runs at ~50 Hz -> "
                     "add up to 20 ms in a sensitivity case."),
        },
        "joint": {"kc": KC, "b": B_VISC, "coul": 0.0},
        "loads": {"turn_deg": TURN_DEG, "pull_up_g": PULL_UP, "rate_deg_s": 25.0, "f_turb_Hz": F_TURB,
                  "alpha_rad_s2": alpha(), "load_factor": load_factor()},
    }
    (OUT / "gimbal_params.json").write_text(json.dumps(params, indent=1))


if __name__ == "__main__":
    main()
