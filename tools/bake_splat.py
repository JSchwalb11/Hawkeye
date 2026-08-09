#!/usr/bin/env python3
"""Bake a Gaussian splat cloud from a triangle mesh.

Hawkeye's fixtures need a world that is not a plane. Planes are honest for
walls and floors, and they are what the existing fixtures use, but a plane
cannot answer "does the map look like the thing it mapped" -- every plane looks
like every other plane. A real object can.

This produces a `.splat` file in the widely used 32-bytes-per-splat layout
(position f32x3, scale f32x3, colour u8x4, rotation quaternion u8x4). Each
splat is an anisotropic Gaussian flattened along the local surface normal,
which is what a splat trained on a solid object converges to and what makes
its depth well defined.

To be explicit about provenance: this is a splat *baked from a mesh*, not a
photogrammetric capture. The distinction matters and is not papered over
anywhere -- the file it writes is a genuine Gaussian splat cloud, and the
fixtures ray-cast against the Gaussians rather than against the triangles, but
its shape fidelity is the source mesh's, not a camera's.

Usage:
    tools/bake_splat.py in.obj out.splat --height 93 --count 40000
"""

import argparse
import math
import random
import struct
import sys


def load_obj(path):
    verts = []
    faces = []
    for line in open(path, "r", errors="replace"):
        if line.startswith("v "):
            p = line.split()
            verts.append((float(p[1]), float(p[2]), float(p[3])))
        elif line.startswith("f "):
            idx = []
            for tok in line.split()[1:]:
                s = tok.split("/")[0]
                if not s:
                    continue
                i = int(s)
                idx.append(i - 1 if i > 0 else len(verts) + i)
            # Fan-triangulate anything with more than three corners.
            for k in range(1, len(idx) - 1):
                faces.append((idx[0], idx[k], idx[k + 1]))
    return verts, faces


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 1.0)


def basis_from_normal(n):
    """An orthonormal (u, v) spanning the plane perpendicular to n."""
    ref = (0.0, 0.0, 1.0) if abs(n[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = norm(cross(n, ref))
    v = cross(n, u)
    return u, v


def quat_from_basis(u, v, n):
    """Quaternion (w, x, y, z) for the rotation whose columns are u, v, n."""
    m = [[u[0], v[0], n[0]],
         [u[1], v[1], n[1]],
         [u[2], v[2], n[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n_ = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n_, x / n_, y / n_, z / n_)


def colour_for(height_frac):
    """Weathered copper above the pedestal, granite below.

    Colour is not scored anywhere -- the fixtures measure geometry -- but a
    splat with a plausible colour is a splat someone can open in any viewer and
    recognise, which is worth the four bytes.
    """
    if height_frac < 0.50:
        g = 120 + int(30 * height_frac)
        return (g, g - 4, g - 10, 255)          # granite pedestal
    return (94, 160, 141, 255)                   # oxidised copper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("out")
    ap.add_argument("--height", type=float, default=93.0,
                    help="target height in metres for the whole model")
    ap.add_argument("--count", type=int, default=40000)
    ap.add_argument("--seed", type=int, default=20260808)
    ap.add_argument("--flatten", type=float, default=6.0,
                    help="ratio of tangential to normal Gaussian extent")
    ap.add_argument("--tri", default=None,
                    help="also write the placed triangles, for exact scoring")
    args = ap.parse_args()

    verts, faces = load_obj(args.obj)
    if not faces:
        sys.exit("no triangles in %s" % args.obj)

    # Put the model upright in metres with its base at z = 0 and its centre of
    # mass over the origin, so a fixture can place it without a second frame.
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    ext = [hi[i] - lo[i] for i in range(3)]
    up = ext.index(max(ext))
    scale = args.height / ext[up]

    def place(p):
        q = [(p[i] - (lo[i] + hi[i]) * 0.5) * scale for i in range(3)]
        q[up] = (p[up] - lo[up]) * scale
        # Emit as (x, y, z) with z up, whichever axis the source called up.
        order = [i for i in range(3) if i != up] + [up]
        return (q[order[0]], q[order[1]], q[order[2]])

    tris = []
    total_area = 0.0
    for (a, b, c) in faces:
        pa, pb, pc = place(verts[a]), place(verts[b]), place(verts[c])
        n = cross(sub(pb, pa), sub(pc, pa))
        area = 0.5 * math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        if area <= 1e-9:
            continue
        tris.append((pa, pb, pc, norm(n), area))
        total_area += area

    if total_area <= 0.0:
        sys.exit("degenerate mesh")

    # Area-weighted sampling: a splat cloud with the same number of Gaussians on
    # a large flat panel as on a small detail would misrepresent both.
    cum = []
    acc = 0.0
    for t in tris:
        acc += t[4]
        cum.append(acc)

    rng = random.Random(args.seed)
    spacing = math.sqrt(total_area / args.count)
    sigma_t = spacing * 0.75
    sigma_n = sigma_t / args.flatten

    zlo = min(min(t[0][2], t[1][2], t[2][2]) for t in tris)
    zhi = max(max(t[0][2], t[1][2], t[2][2]) for t in tris)
    zspan = max(zhi - zlo, 1e-6)

    def pick():
        r = rng.random() * acc
        lo_i, hi_i = 0, len(cum) - 1
        while lo_i < hi_i:
            mid = (lo_i + hi_i) // 2
            if cum[mid] < r:
                lo_i = mid + 1
            else:
                hi_i = mid
        return tris[lo_i]

    # The triangles the splats were sampled from, in the same placed frame.
    # A splat cloud cannot verify a map to a finer tolerance than its own splat
    # spacing; the triangles can, so centimetre work scores against these and
    # keeps the splat only as what the *sensor* sees.
    if args.tri:
        t = open(args.tri, "wb")
        t.write(b"HKTRI1\0\0")
        t.write(struct.pack("<I", len(tris)))
        t.write(struct.pack("<I", 0))
        for (pa, pb, pc, _n, _a) in tris:
            for p in (pa, pb, pc):
                t.write(struct.pack("<3f", *p))
        t.close()
        print("%s: %d triangles" % (args.tri, len(tris)))

    out = open(args.out, "wb")
    for _ in range(args.count):
        pa, pb, pc, n, _area = pick()
        r1, r2 = rng.random(), rng.random()
        s = math.sqrt(r1)
        w0, w1, w2 = 1.0 - s, s * (1.0 - r2), s * r2
        p = tuple(pa[i] * w0 + pb[i] * w1 + pc[i] * w2 for i in range(3))

        u, v = basis_from_normal(n)
        q = quat_from_basis(u, v, n)
        col = colour_for((p[2] - zlo) / zspan)

        out.write(struct.pack("<3f", *p))
        out.write(struct.pack("<3f", sigma_t, sigma_t, sigma_n))
        out.write(struct.pack("<4B", *col))
        out.write(struct.pack("<4B", *[max(0, min(255, int(round(c * 128.0)) + 128))
                                       for c in q]))
    out.close()

    print("%s: %d splats, %.1f m tall, %.1f m^2 surface, "
          "sigma %.3f m tangential / %.3f m normal"
          % (args.out, args.count, args.height, total_area, sigma_t, sigma_n))


if __name__ == "__main__":
    main()
