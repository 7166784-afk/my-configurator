#!/usr/bin/env python3
"""
Perspex (acrylic) model of a medical wagon - laser-cut parts with tab & slot joints.

All dimensions in millimetres. Change the parameters below and re-run:
    python3 generate.py
Outputs (next to this script):
    wagon_cut.svg     - cut file (all parts nested on one sheet), mm units
    wagon_cut.dxf     - same, DXF (layer CUT)
    wagon_labels.svg  - same sheet with part letters, for orientation while assembling
    wagon_3d.svg      - 3D preview of the assembled model
It also checks in 3D that no two parts overlap (every tab passes through its slot).

3D coordinate system: X = width (left -> right), Y = height, Z = depth (front=0 -> back).
X=0 is the inner face of the left side panel.
"""
import math, os, sys
from shapely.geometry import Polygon, box, Point
from shapely.ops import unary_union
from shapely import affinity

# ---------------------------------------------------------------- parameters
T = 4.0          # sheet thickness
C = 0.15         # extra slot clearance (slot = T + C wide, tab + C long)
H = 110.0        # overall height (side panels)
W = 95.0         # overall width (outside of side panels)
D = 60.0         # overall depth
WI = W - 2 * T   # interior width = 87
ZB = D - 2 * T   # back panel front face (back panel inset 4 mm from rear edge) = 52

Y_BOTTOM, Y_MIDDLE, Y_UPPER, Y_TOP = 4.0, 40.0, 72.0, 96.0   # shelf bottom faces
X_PART = 48.0                    # partition left face (drawer opening = 0..48)
PLAT_W = X_PART + 2 * T          # top platform width = 56
Y_OXY = [12.0, 46.0, 80.0]       # oxygen-side shelves (bottom faces)
OXY_OUT, OXY_DEPTH = 28.0, 26.0  # oxygen shelf size (outward x depth)
Z_RIB = OXY_DEPTH                # rib behind the oxygen shelves, z = 26..30

# drawer
DW, DD = 47.0, 51.0              # drawer outer width / depth
D_FRONT_H, D_WALL_H = 19.0, 15.0
D_X0 = 0.5                       # drawer left gap
D_SIDE_X = [3.0, DW - 3.0 - T]   # drawer side walls x (drawer-local)
D_BOT_Y = 3.0                    # drawer bottom height (drawer-local)

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- helpers
def rect(x0, y0, x1, y1):
    return box(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))

def slot(x0, y0, x1, y1):
    """Slot rectangle enlarged by C/2 on every side."""
    return rect(x0, y0, x1, y1).buffer(C / 2, join_style=2)

def scallop_top(x0, x1, base, n, rise):
    """Polygon of n circular bumps on top of the line y=base between x0..x1."""
    pts = [(x0, base)]
    w = (x1 - x0) / n
    r = (w * w / 4 + rise * rise) / (2 * rise)
    for i in range(n):
        a, b = x0 + i * w, x0 + (i + 1) * w
        cx, cy = (a + b) / 2, base + rise - r
        a0 = math.atan2(base - cy, a - cx)
        a1 = math.atan2(base - cy, b - cx)
        steps = 24
        for k in range(1, steps + 1):
            t = a0 + (a1 - a0) * k / steps
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    pts.append((x1, base))
    pts.append((x1, base - 1))
    pts.append((x0, base - 1))
    return Polygon(pts).buffer(0)

def rounded_corner_cut(poly, cx, cy, r, qx, qy):
    """Round a convex corner at (cx,cy); (qx,qy) = direction into the part (+-1)."""
    sq = rect(cx, cy, cx + qx * r, cy + qy * r)
    circ = Point(cx + qx * r, cy + qy * r).buffer(r, 64)
    return poly.difference(sq.difference(circ))

class Part:
    def __init__(self, name, label, shape, place, qty=1):
        self.name, self.label, self.shape, self.place, self.qty = name, label, shape, place, qty

parts = []

# ---------------------------------------------------------------- tab positions (shared)
SHELF_DEPTH = ZB                         # 52
SIDE_TABS_Z = [(8, 20), (32, 44)]        # shelf tabs into side panels (z ranges)
BACK_TABS_X_FULL = [(12, 28), (59, 75)]  # shelf tabs into back panel (x ranges)
BACK_TABS_X_PLAT = [(10, 22), (30, 42)]  # platform tabs into back panel
PART_TABS_Z = [(10, 20), (32, 42)]       # partition tabs into upper shelf / platform
BACKPANEL_TABS_Y = [(20, 32), (52, 62), (80, 90)]   # back panel tabs into sides
PART_BACK_TAB_Y = (Y_UPPER + T + 5, Y_TOP - 5)       # partition tab into back panel
OXY_TAB_Z = (5, 21)
RIB_TABS_Y = [(26, 38), (58, 70)]
OXY_RIB_TAB_X = (6, 18)                  # distance outward from side panel outer face
RIB_Y0, RIB_Y1 = Y_OXY[0] - 4, Y_OXY[-1] + T + 8

# ---------------------------------------------------------------- side panels (local u=z, v=y)
def side_panel(left):
    p = unary_union([rect(0, 0, D, H - 6), scallop_top(0, D, H - 6, 3, 6)])
    p = rounded_corner_cut(p, 0, 0, 3, 1, 1)
    p = rounded_corner_cut(p, D, 0, 3, -1, 1)
    cuts = []
    shelves = [Y_BOTTOM, Y_MIDDLE, Y_UPPER] + ([Y_TOP] if left else [])
    for ys in shelves:
        for z0, z1 in SIDE_TABS_Z:
            cuts.append(slot(z0, ys, z1, ys + T))
    for y0, y1 in BACKPANEL_TABS_Y:
        cuts.append(slot(ZB, y0, ZB + T, y1))
    if left:
        for yo in Y_OXY:
            cuts.append(slot(OXY_TAB_Z[0], yo, OXY_TAB_Z[1], yo + T))
        for y0, y1 in RIB_TABS_Y:
            cuts.append(slot(Z_RIB, y0, Z_RIB + T, y1))
    return p.difference(unary_union(cuts))

# placement: local (u,v,w) -> world. axes given as world vectors, origin world point.
def place(ox, oy, oz, U, V, Wv):
    return (ox, oy, oz, U, V, Wv)
EX, EY, EZ = (1, 0, 0), (0, 1, 0), (0, 0, 1)

parts.append(Part("Left side", "A", side_panel(True), place(-T, 0, 0, EZ, EY, EX)))
parts.append(Part("Right side", "B", side_panel(False), place(WI, 0, 0, EZ, EY, EX)))

# ---------------------------------------------------------------- back panel (local u=x, v=y)
def back_panel():
    p = unary_union([rect(0, 0, WI, H - 6), scallop_top(0, WI, H - 6, 4, 6)])
    tabs = []
    for y0, y1 in BACKPANEL_TABS_Y:
        tabs += [rect(-T, y0, 0, y1), rect(WI, y0, WI + T, y1)]
    p = unary_union([p] + tabs)
    cuts = []
    for ys in [Y_BOTTOM, Y_MIDDLE, Y_UPPER]:
        for x0, x1 in BACK_TABS_X_FULL:
            cuts.append(slot(x0, ys, x1, ys + T))
    for x0, x1 in BACK_TABS_X_PLAT:
        cuts.append(slot(x0, Y_TOP, x1, Y_TOP + T))
    cuts.append(slot(X_PART, PART_BACK_TAB_Y[0], X_PART + T, PART_BACK_TAB_Y[1]))
    return p.difference(unary_union(cuts))

parts.append(Part("Back panel", "C", back_panel(), place(0, 0, ZB, EX, EY, EZ)))

# ---------------------------------------------------------------- shelves (local u=x, v=z)
def shelf(width, left_tab=True, right_tab=True, back_tabs=BACK_TABS_X_FULL):
    p = rect(0, 0, width, SHELF_DEPTH)
    tabs = []
    for z0, z1 in SIDE_TABS_Z:
        if left_tab:
            tabs.append(rect(-T, z0, 0, z1))
        if right_tab:
            tabs.append(rect(width, z0, width + T, z1))
    for x0, x1 in back_tabs:
        tabs.append(rect(x0, SHELF_DEPTH, x1, SHELF_DEPTH + T))
    return unary_union([p] + tabs)

bottom = shelf(WI)
# big front cut-out on the left of the middle shelf, as in the photo:
# from x=6 to x=40, 28 mm deep, large round corner on the left, small one on the right
MID_CUT_X0, MID_CUT_X1, MID_CUT_DEPTH = 6.0, 40.0, 28.0
mid_cut = rect(MID_CUT_X0, -1, MID_CUT_X1, MID_CUT_DEPTH)
mid_cut = rounded_corner_cut(mid_cut, MID_CUT_X0, MID_CUT_DEPTH, 16, 1, -1)
mid_cut = rounded_corner_cut(mid_cut, MID_CUT_X1, MID_CUT_DEPTH, 4, -1, -1)
middle = shelf(WI).difference(mid_cut)
upper = shelf(WI).difference(unary_union([slot(X_PART, z0, X_PART + T, z1) for z0, z1 in PART_TABS_Z]))
plat = shelf(PLAT_W, right_tab=False, back_tabs=BACK_TABS_X_PLAT)
plat = rounded_corner_cut(plat, PLAT_W, 0, 4, -1, 1)
plat = plat.difference(unary_union([slot(X_PART, z0, X_PART + T, z1) for z0, z1 in PART_TABS_Z]))

# horizontal parts: local u -> X, local v -> Z, extrusion -> Y
parts.append(Part("Bottom shelf", "D", bottom, place(0, Y_BOTTOM, 0, EX, EZ, EY)))
parts.append(Part("Middle shelf", "E", middle, place(0, Y_MIDDLE, 0, EX, EZ, EY)))
parts.append(Part("Upper shelf", "F", upper, place(0, Y_UPPER, 0, EX, EZ, EY)))
parts.append(Part("Top platform", "G", plat, place(0, Y_TOP, 0, EX, EZ, EY)))

# ---------------------------------------------------------------- partition (local u=z, v=y)
y_lo, y_hi = Y_UPPER + T, Y_TOP
part = rect(0, y_lo, SHELF_DEPTH, y_hi)
ptabs = []
for z0, z1 in PART_TABS_Z:
    ptabs += [rect(z0, y_lo - T, z1, y_lo), rect(z0, y_hi, z1, y_hi + T)]
ptabs.append(rect(SHELF_DEPTH, PART_BACK_TAB_Y[0], SHELF_DEPTH + T, PART_BACK_TAB_Y[1]))
part = unary_union([part] + ptabs)
parts.append(Part("Drawer partition", "H", part, place(X_PART, 0, 0, EZ, EY, EX)))

# ---------------------------------------------------------------- oxygen side shelves + rib
# local u = distance outward from left panel's outer face (world x = -T - u), v = z
def oxy_shelf(hole):
    p = rect(0, 0, OXY_OUT, OXY_DEPTH)
    p = rounded_corner_cut(p, OXY_OUT, 0, 6, -1, 1)
    p = unary_union([p, rect(-T, OXY_TAB_Z[0], 0, OXY_TAB_Z[1]),
                     rect(OXY_RIB_TAB_X[0], OXY_DEPTH, OXY_RIB_TAB_X[1], OXY_DEPTH + T)])
    if hole:
        p = p.difference(Point(OXY_OUT / 2 + 1, OXY_DEPTH / 2 - 1).buffer(7, 64))
    return p

for i, yo in enumerate(Y_OXY):
    parts.append(Part(f"Cylinder shelf {i+1}", f"J{i+1}", oxy_shelf(hole=(i > 0)),
                      place(-T, yo, 0, (-1, 0, 0), EZ, EY)))

rib = rect(0, RIB_Y0, OXY_OUT - 6, RIB_Y1)
rib = rounded_corner_cut(rib, OXY_OUT - 6, RIB_Y0, 5, -1, 1)
rib = rounded_corner_cut(rib, OXY_OUT - 6, RIB_Y1, 5, -1, -1)
rib = unary_union([rib] + [rect(-T, y0, 0, y1) for y0, y1 in RIB_TABS_Y])
rib = rib.difference(unary_union([slot(OXY_RIB_TAB_X[0], yo, OXY_RIB_TAB_X[1], yo + T) for yo in Y_OXY]))
parts.append(Part("Cylinder rib", "K", rib, place(-T, 0, Z_RIB, (-1, 0, 0), EY, EZ)))

# ---------------------------------------------------------------- drawer
DS_TAB_Y = (8.0, 12.0)                    # side wall end tabs into front/back
DB_X = (D_SIDE_X[0] + T, D_SIDE_X[1])     # bottom between side walls: 7..40
DB_Z = (T, DD - T)                        # bottom between front/back: 4..47
DB_TABS_X = [(12, 20), (27, 35)]          # bottom tabs into front/back walls
DB_TABS_Z = [(10, 20), (31, 41)]          # bottom tabs into side walls (drawer z)

def drawer_end(front):
    h = D_FRONT_H if front else D_WALL_H
    p = rect(0, 0, DW, h)
    if front:  # finger pull
        p = p.difference(Polygon([(DW/2 - 9, h + 1), (DW/2 + 9, h + 1), (DW/2 + 6, h - 5), (DW/2 - 6, h - 5)]))
    cuts = [slot(x, DS_TAB_Y[0], x + T, DS_TAB_Y[1]) for x in D_SIDE_X]
    cuts += [slot(x0, D_BOT_Y, x1, D_BOT_Y + T) for x0, x1 in DB_TABS_X]
    return p.difference(unary_union(cuts))

def drawer_side():  # local u = z, v = y
    p = rect(T, 0, DD - T, D_WALL_H)
    p = unary_union([p, rect(0, DS_TAB_Y[0], T, DS_TAB_Y[1]), rect(DD - T, DS_TAB_Y[0], DD, DS_TAB_Y[1])])
    return p.difference(unary_union([slot(z0, D_BOT_Y, z1, D_BOT_Y + T) for z0, z1 in DB_TABS_Z]))

def drawer_bottom():  # local u = x, v = z
    p = rect(DB_X[0], DB_Z[0], DB_X[1], DB_Z[1])
    tabs = []
    for x0, x1 in DB_TABS_X:
        tabs += [rect(x0, 0, x1, T), rect(x0, DD - T, x1, DD)]
    for z0, z1 in DB_TABS_Z:
        tabs += [rect(DB_X[0] - T, z0, DB_X[0], z1), rect(DB_X[1], z0, DB_X[1] + T, z1)]
    return unary_union([p] + tabs)

dy = Y_UPPER + T
parts.append(Part("Drawer front", "L1", drawer_end(True), place(D_X0, dy, 0, EX, EY, EZ)))
parts.append(Part("Drawer back", "L2", drawer_end(False), place(D_X0, dy, DD - T, EX, EY, EZ)))
parts.append(Part("Drawer side L", "L3", drawer_side(), place(D_X0 + D_SIDE_X[0], dy, 0, EZ, EY, EX)))
parts.append(Part("Drawer side R", "L4", drawer_side(), place(D_X0 + D_SIDE_X[1], dy, 0, EZ, EY, EX)))
parts.append(Part("Drawer bottom", "L5", drawer_bottom(), place(D_X0, dy + D_BOT_Y, 0, EX, EZ, EY)))

# ---------------------------------------------------------------- 3D check
def part_mesh(p):
    import trimesh, numpy as np
    m = trimesh.creation.extrude_polygon(p.shape, T)
    ox, oy, oz, U, V, Wv = p.place
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = U, V, Wv, (ox, oy, oz)
    m.apply_transform(M)
    return m

def check_3d():
    import trimesh
    meshes = [part_mesh(p) for p in parts]
    bad = 0
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            a, b = meshes[i], meshes[j]
            lo = [max(a.bounds[0][k], b.bounds[0][k]) for k in range(3)]
            hi = [min(a.bounds[1][k], b.bounds[1][k]) for k in range(3)]
            if any(lo[k] >= hi[k] for k in range(3)):
                continue
            inter = trimesh.boolean.intersection([a, b], engine="manifold")
            if inter.volume > 1e-3:
                bad += 1
                print(f"  OVERLAP {parts[i].name} x {parts[j].name}: {inter.volume:.2f} mm3")
    print("3D overlap check:", "OK - no parts collide" if bad == 0 else f"{bad} collisions")
    return meshes, bad

# ---------------------------------------------------------------- nesting & export
def layout(gap=3.0, sheet_w=300.0):
    placed, x, y, row_h = [], gap, gap, 0
    order = sorted(parts, key=lambda p: -(p.shape.bounds[3] - p.shape.bounds[1]))
    for p in order:
        minx, miny, maxx, maxy = p.shape.bounds
        w, h = maxx - minx, maxy - miny
        if x + w + gap > sheet_w:
            x, y, row_h = gap, y + row_h + gap, 0
        # SVG y axis points down: flip parts so they read upright
        s = affinity.scale(p.shape, 1, -1, origin=(0, 0))
        s = affinity.translate(s, x - minx, y + maxy)
        placed.append((p, s))
        x += w + gap
        row_h = max(row_h, h)
    return placed, sheet_w, y + row_h + gap

def ring_path(coords):
    pts = list(coords)
    return "M " + " L ".join(f"{a:.3f},{b:.3f}" for a, b in pts[:-1]) + " Z"

def shape_paths(s):
    geoms = getattr(s, "geoms", [s])
    out = []
    for g in geoms:
        out.append(ring_path(g.exterior.coords))
        out += [ring_path(r.coords) for r in g.interiors]
    return out

def write_svg(placed, sw, sh, fname, labels):
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{sw:.1f}mm" height="{sh:.1f}mm" viewBox="0 0 {sw:.1f} {sh:.1f}">',
             '<g id="CUT" fill="none" stroke="#ff0000" stroke-width="0.1">']
    for p, s in placed:
        for d in shape_paths(s):
            lines.append(f'<path d="{d}"/>')
    lines.append('</g>')
    if labels:
        lines.append('<g id="LABELS" fill="#0000ff" font-family="Arial" font-size="5" text-anchor="middle">')
        for p, s in placed:
            c = s.representative_point()
            lines.append(f'<text x="{c.x:.1f}" y="{c.y + 2:.1f}">{p.label}</text>')
        lines.append('</g>')
    lines.append('</svg>')
    open(os.path.join(HERE, fname), "w").write("\n".join(lines))

def write_dxf(placed):
    import ezdxf
    doc = ezdxf.new(units=4)  # mm
    doc.layers.add("CUT", color=1)
    msp = doc.modelspace()
    for p, s in placed:
        geoms = getattr(s, "geoms", [s])
        for g in geoms:
            for ring in [g.exterior] + list(g.interiors):
                pts = [(a, -b) for a, b in list(ring.coords)[:-1]]  # back to y-up
                msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": "CUT"})
    doc.saveas(os.path.join(HERE, "wagon_cut.dxf"))

PREVIEW_AZ = 35   # camera at front-left

def write_preview(meshes):
    """Flat-shaded isometric render of the assembly into an SVG."""
    import numpy as np
    az, el = math.radians(PREVIEW_AZ), math.radians(22)
    R1 = np.array([[math.cos(az), 0, math.sin(az)], [0, 1, 0], [-math.sin(az), 0, math.cos(az)]])
    R2 = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    R = R2 @ R1
    light = np.array([-0.4, 0.7, 0.6]); light /= np.linalg.norm(light)
    colors = {"A": (150, 190, 230), "B": (150, 190, 230), "C": (150, 190, 230), "H": (150, 190, 230),
              "K": (150, 190, 230)}
    polys = []
    for p, m in zip(parts, meshes):
        base = colors.get(p.label, (110, 160, 120) if p.label[0] in "DEFGJ" else (175, 205, 240))
        import trimesh
        vv, ff = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=2.5)
        m = trimesh.Trimesh(vv, ff, process=False)
        F = np.array([1, 1, -1])  # world Z points away from the front viewer -> flip to right-handed view space
        v = ((m.vertices - [WI / 2, H / 2, D / 2]) * F) @ R.T
        for f, n in zip(m.faces, m.face_normals):
            nn = R @ (n * F)
            if nn[2] <= 0:
                continue  # back-facing
            shade = 0.45 + 0.55 * max(0.0, float(np.dot(nn, light)))
            col = tuple(int(min(255, c * shade)) for c in base)
            tri = v[f]
            polys.append((tri[:, 2].mean(), tri, col))
    polys.sort(key=lambda t: t[0])
    s, cx, cy = 4.0, 300, 300
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="600" height="600" viewBox="0 0 600 600">',
           '<rect width="600" height="600" fill="#ffffff"/>']
    for _, tri, col in polys:
        pts = " ".join(f"{cx + a * s:.1f},{cy - b * s:.1f}" for a, b, _ in tri)
        out.append(f'<polygon points="{pts}" fill="rgb{col}" stroke="rgb{col}" stroke-width="0.4"/>')
    out.append('</svg>')
    open(os.path.join(HERE, "wagon_3d.svg"), "w").write("\n".join(out))

if __name__ == "__main__":
    for p in parts:
        if not p.shape.is_valid or p.shape.geom_type != "Polygon":
            print("WARNING: part not a single polygon:", p.name, p.shape.geom_type)
    meshes, bad = check_3d()
    placed, sw, sh = layout()
    write_svg(placed, sw, sh, "wagon_cut.svg", labels=False)
    write_svg(placed, sw, sh, "wagon_labels.svg", labels=True)
    write_dxf(placed)
    write_preview(meshes)
    print(f"{len(parts)} parts, sheet {sw:.0f} x {sh:.0f} mm")
    sys.exit(1 if bad else 0)
