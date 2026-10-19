"""
Small 3D name signs for the stations.

Swift has no 3D text, so each label is a mesh: the text is drawn with Pillow
(using the DejaVu font that ships with matplotlib), the inked pixels are merged
into rectangles, and the rectangles are extruded into one STL file per label.
One mesh per label keeps Swift fast (it re-sends every shape each step). The
files are generated once into the temp directory and reused.

A label either stands upright on a thin plate, turned to face the default
camera, or lies flat on the table (for busy spots where an upright sign would
be in a robot's way), turned so it reads from the camera.
"""
import hashlib
import struct
import tempfile
from math import atan2, pi
from pathlib import Path

import matplotlib
import numpy as np
import spatialgeometry as geometry
from PIL import Image, ImageDraw, ImageFont
from spatialmath import SE3

FONT = Path(matplotlib.get_data_path()) / "fonts" / "ttf" / "DejaVuSans-Bold.ttf"
CACHE = Path(tempfile.gettempdir()) / "barista_labels"
FONT_PX = 48                 # render size; the mesh is scaled to the asked height
DEPTH = 0.002                # how far the letters stand off the plate
PLATE_T = 0.004
PAD = 0.25                   # plate margin, as a fraction of the text height
TEXT_COLOR = (0.98, 0.95, 0.80, 1)
PLATE_COLOR = (0.10, 0.10, 0.12, 1)


# ---------------------------------------------------------------- text -> mesh
def _bitmap(text):
    font = ImageFont.truetype(str(FONT), FONT_PX)
    left, top, right, bottom = font.getbbox(text)
    img = Image.new("L", (right - left + 2, FONT_PX + 4), 0)
    ImageDraw.Draw(img).text((1 - left, 2), text, fill=255, font=font)
    return np.array(img) > 127


def _rectangles(ink):
    """Cover the inked pixels with rectangles (row runs, merged downwards)."""
    rows, cols = ink.shape
    used = np.zeros_like(ink)
    rects = []
    for r in range(rows):
        c = 0
        while c < cols:
            if not ink[r, c] or used[r, c]:
                c += 1
                continue
            c1 = c
            while c1 < cols and ink[r, c1] and not used[r, c1]:
                c1 += 1
            r1 = r + 1
            while r1 < rows and ink[r1, c:c1].all() and not used[r1, c:c1].any():
                r1 += 1
            used[r:r1, c:c1] = True
            rects.append((c, c1, r, r1))
            c = c1
    return rects


def _box_triangles(x0, x1, y0, y1, z0, z1):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [v[list(f)] for f in faces]


def _write_stl(path, triangles):
    with open(path, "wb") as f:
        f.write(b"barista label".ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *tri[0], *tri[1], *tri[2], 0))


def text_mesh(text, height):
    """STL of `text`, `height` m tall, in its own frame: reading along +X,
    letters up +Z, standing out of the plate towards -Y. Returns (path, width)."""
    ink = _bitmap(text)
    scale = height / FONT_PX
    width = ink.shape[1] * scale
    key = hashlib.md5(f"{text}|{height}|{FONT_PX}|{DEPTH}".encode()).hexdigest()[:12]
    path = CACHE / f"label_{key}.stl"
    if not path.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        rows = ink.shape[0]
        tris = []
        for c0, c1, r0, r1 in _rectangles(ink):
            tris += _box_triangles(c0 * scale, c1 * scale, -DEPTH, 0.0,
                                   (rows - r1) * scale, (rows - r0) * scale)
        _write_stl(path, tris)
    return path, width, ink.shape[0] * scale


# ---------------------------------------------------------------- signs
def sign_pose(at, camera, flat=False):
    """Pose of a label's frame at `at` (its centre, bottom edge for upright
    signs) so it reads from `camera`."""
    x, y, z = at
    dx, dy = camera[0] - x, camera[1] - y
    if not flat:
        # The text faces local -Y, so turn -Y towards the camera.
        return SE3(x, y, z) * SE3.Rz(atan2(dy, dx) + pi / 2)
    # Lying on the table: the tops of the letters point away from the camera.
    d = np.array([-dx, -dy]) / np.hypot(dx, dy)
    return SE3(x, y, z) * SE3.Rz(atan2(-d[0], d[1])) * SE3.Rx(-pi / 2)


def layout_sign(text, at, camera, height=0.03, flat=False):
    """(letters mesh path, letters pose, plate pose, plate size) for a label.

    `at` is the bottom centre of an upright sign, or the centre of a flat one
    on the surface it lies on.
    """
    path, w, h = text_mesh(text, height)
    pad = PAD * height
    T = sign_pose(at, camera, flat)
    if flat:
        # Centre it on `at`, and lift it so the plate's back sits on the surface.
        T = SE3(0.0, 0.0, PLATE_T) * T * SE3(0.0, 0.0, -h / 2)
    T = T * SE3(-w / 2, 0.0, 0.0 if flat else pad)
    size = np.array([w + 2 * pad, PLATE_T + DEPTH, h + 2 * pad])
    return path, T, T * SE3(w / 2, (PLATE_T - DEPTH) / 2, h / 2), size


def make_sign(text, at, camera, height=0.03, flat=False):
    """Shapes for one label: [plate, letters]."""
    path, T, T_box, size = layout_sign(text, at, camera, height, flat)
    # T_box centres the plate and letters together; the plate alone sits
    # DEPTH/2 further back.
    plate = geometry.Cuboid(scale=[size[0], PLATE_T, size[2]], color=PLATE_COLOR,
                            pose=T_box * SE3(0.0, DEPTH / 2, 0.0))
    letters = geometry.Mesh(filename=str(path), color=TEXT_COLOR, pose=T)
    return [plate, letters]
