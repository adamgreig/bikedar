# Microstrip patch antenna array drawing script.
# Units are metres/kilograms/seconds
# Axis has origin in the top left, x increasing right, y increasing down
# Copyright 2016, 2017 Adam Greig

from __future__ import print_function, division
import datetime
import numpy as np
from scipy.integrate import quad
from scipy.special import jn
from sexp import generate


pcb_h = 0.2
patch_length = 2.82e-3
array_x = 4
array_y = 2


def patch_impedance(w, l, r, h, f0):
    """
    Compute an estimate of the patch impedance, for patch of width w and length
    l, inset feed at distance r, and height above ground h, at frequency f0.
    """
    c0 = 299792458
    l0 = c0 / f0
    k0 = 2 * np.pi / l0
    si = lambda x: quad(lambda t: np.sin(t)/t, 0, x)[0]
    x = k0 * w
    i1 = -2 + np.cos(x) + x * si(x) + np.sin(x)/x
    g1 = i1 / (120 * np.pi**2)
    g12 = 1/(120*np.pi**2) * quad(
        lambda th: (np.sin((k0*w)/2 * np.cos(th))/np.cos(th))**2
        * jn(0, k0 * l * np.sin(th)) * np.sin(th)**3,
        0, np.pi)[0]
    rin0 = 1/(2*(g1 + g12))
    rin = rin0 * np.cos(np.pi * r / l)**2
    print("l0={:.4f} g1={:.4e} g12={:.4e} rin0={:.2f} rin={:.2f}"
          .format(l0, g1, g12, rin0, rin))
    return rin


def microstrip_l(x, x2, y, lspec, h):
    """
    Draw a microstrip upside-down-L shape from x to x2, starting at y,
    and according to lspec (a list of two lists; the first a list of vertical
    segments and the second a list of horizontal segments, each segment with a
    width and a length).

    Returns a list of zones and the new height.
    """

    # Store generated strips
    strips = []

    # Emit a strip whenever the current width changes
    w = None

    # Store points to draw current strip between
    points = [(x, y)]

    # Compute the horizontal direction
    hdir = 1 if x2 > x else -1

    # Process each bit of vertical strip
    for vspec in lspec[0]:
        if w is None:
            w = vspec[0]
        elif w != vspec[0]:
            strips.append(generate_feedline(points, w, h))
            points = [(x, y)]
            w = vspec[0]
        y -= vspec[1]
        points.append((x, y))

    # Process the horizontal strips
    for hspec in lspec[1]:
        if w is None:
            w = hspec[0]
        elif w != hspec[0]:
            strips.append(generate_feedline(points, w, h))
            points = [(x, y)]
            w = hspec[0]
        dx = hspec[1]
        if dx is None:
            dx = abs(x2 - x)
        x += dx * hdir
        points.append((x, y))

    # Generate last strip
    points.append((x, y - w/2))
    strips.append(generate_feedline(points, w, h))

    return strips, y


def direction(pa, pb):
    """Find the direction between two points."""
    v = pb[0] - pa[0], pb[1] - pa[1]
    l = np.sqrt(v[0]**2 + v[1]**2)
    return v[0]/l, v[1]/l


def convex(v1, v2):
    """See if (v1, v2) are convex."""
    return (-v1[1] * v2[0] + v1[0]*v2[1]) < 0


def generate_feedline(points, w, h):
    """
    Generate microstrip feedline passing through points of width w [m].
    Specify the height above dielectric h [m] for mitreing purposes.
    Returns a zone definition (list of zone corners).
    """

    assert len(points) >= 2, "Must have at least two points to draw feedline"

    # Cop out if there are only two points (easy mode)
    if len(points) == 2:
        pa, pb = points
        v = direction(pa, pb)
        return [
            (pa[0] + v[1]*w/2, pa[1] + v[0]*w/2),
            (pb[0] + v[1]*w/2, pb[1] + v[0]*w/2),
            (pb[0] - v[1]*w/2, pb[1] - v[0]*w/2),
            (pa[0] - v[1]*w/2, pa[1] - v[0]*w/2),
            (pa[0] + v[1]*w/2, pa[1] + v[0]*w/2)
        ]

    # Compute mitre related constants for three or more points (hard mode)
    # We put a fudge factor of 0.8 in to make the mitres a bit less thin.
    assert w/h >= 1/4, "Microstrip W/H must >= 1/4 for mitres to be correct"
    m = (52 + 65 * np.exp(-27/20 * w/h)) / 100
    d = w * np.sqrt(2)
    x = 0.8 * (d * m)
    a = x * np.sqrt(2)

    line = []

    def emit_corner(pa, pb, pc):
        """
        Compute where to stick a zone corner for the three points along a line.
        The corner will be near pb, but is mostly w/2 away from it to get the
        right track width.
        For concave corners, we simply stick the corner at the point which is
        w/2 away from pb perpendicular to pa->pb and w/2 away in the pb->pa
        direction (in other words, this is the inside corner in the pa-pb-pc
        angle, a distance sqrt(2)*w/2 from pb).
        For convex corners we must mitre, so take the previous point and first
        pull it in the pb->pa direction by a, and then make a second point
        pushed in the pb->pc direction (perpendicular to pb->pa) by a.
        The parameter a is as per Douville and James "Experimental study of
        symmetric microstrip bends and their compensation", 1978, IEEE Trans
        Microwave Theory Tech.
        """
        v1, v2 = direction(pa, pb), direction(pb, pc)
        if convex(v1, v2):
            line.append((pb[0] - v1[1]*w/2 + v1[0]*(w/2 - a),
                         pb[1] + v1[1]*(w/2 - a) + v1[0]*w/2))
            line.append((pb[0] - v1[1]*(w/2 - a) + v1[0]*w/2,
                         pb[1] + v1[1]*w/2 + v1[0]*(w/2 - a)))
        else:
            line.append((pb[0] - v1[1]*w/2 - v1[0]*w/2,
                         pb[1] - v1[1]*w/2 + v1[0]*w/2))

    # Starting corner by the first point
    v = direction(points[0], points[1])
    line.append((points[0][0] - v[1]*w/2, points[0][1] + v[0]*w/2))

    # Run in the forward direction
    for pa, pb, pc in zip(points, points[1:], points[2:]):
        emit_corner(pa, pb, pc)

    # Two corners at the end of the run
    v = direction(points[-2], points[-1])
    line.append((points[-1][0] - v[1]*w/2, points[-1][1] + v[0]*w/2))
    line.append((points[-1][0] + v[1]*w/2, points[-1][1] - v[0]*w/2))

    # Now run back to the start, doing the other side of the track
    for pa, pb, pc in zip(points[::-1], points[-2::-1], points[-3::-1]):
        emit_corner(pa, pb, pc)

    # Final point and close the loop
    v = direction(points[0], points[1])
    line.append((points[0][0] + v[1]*w/2, points[0][1] - v[0]*w/2))
    line.append((points[0][0] - v[1]*w/2, points[0][1] + v[0]*w/2))

    return line


def generate_patch(w, l, w_inset=None, l_inset=None, r_corner=None):
    """
    Draw a patch of width w [m] and length l [m],
    with optional inset feedpoint width w_inset [m] and length l_inset [m],
    and optional opposite corner truncation of length r_corner [m].
    Returns a zone definition (list of zone corners).
    """

    patch = [(-w/2, -l/2)]

    if r_corner:
        patch.append((-w/2, l/2 - r_corner))
        patch.append((-w/2 + r_corner, l/2))
    else:
        patch.append((-w/2, l/2))

    patch.append((w/2, l/2))

    if r_corner:
        patch.append((w/2, -l/2 + r_corner))
        patch.append((w/2 - r_corner, -l/2))
    else:
        patch.append((w/2, -l/2))

    if w_inset and l_inset:
        patch.append((w_inset/2, -l/2))
        patch.append((w_inset/2, -l/2 + l_inset))
        patch.append((-w_inset/2, -l/2 + l_inset))
        patch.append((-w_inset/2, -l/2))

    patch.append((-w/2, -l/2))

    return patch


def translate(points, x, y):
    """
    Translate a list of points by x and y.
    """
    return [(p[0]+x, p[1]+y) for p in points]


def scale(points, s):
    """
    Scale a list of points by s.
    """
    return [(p[0]*s, p[1]*s) for p in points]


def pcb_lines(points, layer, w=0.1):
    """
    Draw connected lines through points on layer.
    """
    lines = []
    for p, pp in zip(points, points[1:]):
        lines.append([
            "gr_line",
            ["start", p[0], p[1]],
            ["end", pp[0], pp[1]],
            ["layer", layer], ["width", w]
        ])

    return lines


def generate_pcb(feedpoints, cuts, zones, drawings):
    """
    Generate a KiCAD PCB file, with PTH pads located at the feedpoints,
    rectangular board edges defined by cuts, copper zones (which must intersect
    a feedpoint to be filled) defined by lists of points in zones, and drawings
    (list of lists of points).
    """
    # Rescale inputs to millimetres for KiCAD
    feedpoints = scale(feedpoints, 1e3)
    cuts = [scale(cut, 1e3) for cut in cuts]
    zones = [scale(z, 1e3) for z in zones]
    drawings = [scale(d, 1e3) for d in drawings]

    # Draw feedpoints
    modules = []
    for idx, feedpoint in enumerate(feedpoints):
        modules += [
            "module",
            "X{}".format(idx),
            ["layer", "F.Cu"],
            ["tedit", 0],
            ["tstamp", 0],
            ["at", feedpoint[0], feedpoint[1]],
            ["pad",
                1, "thru_hole", "circle", ["at", 0, 0], ["size", 2.5, 2.5],
                ["drill", 1.5], ["layers", "F.Cu"], ["zone_connect", 2],
                ["net", 1, "Antennas"]]],

    # Draw cutouts
    edges = []
    for cut in cuts:
        edges += pcb_lines(cut + [cut[0]], "Edge.Cuts")

    # Draw zones
    kicad_zones = []
    for zone in zones:
        pts = ["pts"]
        for vertex in zone:
            pts.append(["xy", vertex[0], vertex[1]])
        kicad_zones.append([
            "zone",
            ["net", 1],
            ["net_name", "Antennas"],
            ["layer", "F.Cu"],
            ["tstamp", 0],
            ["hatch", "edge", 0.5],
            ["connect_pads", ["clearance", 0.3]],
            ["min_thickness", 0.05],
            ["fill",
                "yes",
                ["arc_segments", 32],
                ["thermal_gap", 0.3],
                ["thermal_bridge_width", 0.25]],
            ["polygon", pts]
        ])

    # Draw drawing lines
    kicad_drawings = []
    for drawing in drawings:
        kicad_drawings += pcb_lines(drawing, "Dwgs.User")

    # Render the PCB
    out = [
        "kicad_pcb",
        ["version", 4],
        ["host", "antennas.py", datetime.datetime.utcnow().isoformat()],
        ["page", "A0"],
        ["layers",
            [0, "F.Cu", "signal"],
            [31, "B.Cu", "signal", "hide"],
            [40, "Dwgs.User", "user"],
            [44, "Edge.Cuts", "user"]],
        ["net", 0, ""],
        ["net", 1, "Antennas"],
        ["net_class", "Default", "",
            ["clearance", 0.2],
            ["trace_width", 0.2],
            ["via_dia", 0.8],
            ["via_drill", 0.4]],
    ] + modules + edges + kicad_drawings + kicad_zones
    return generate(out)


if __name__ == "__main__":
    feedpoints, cutouts, zones = make_arrays(antennas)
    pcb = generate_pcb(feedpoints, cutouts, zones, [])

    with open("antennas.kicad_pcb", "w") as f:
        f.write(pcb)
