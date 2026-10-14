"""Builds the self-collision model of the bimanual controller from the ergoCub URDF collision meshes.

Usage: python make_self_collision_model.py <model.urdf> <out.txt> [--walking]

With --walking the legs are swept over LEG_SWEEP (walking-controller), otherwise they are at zero.

Every part is a set of links merged in one frame of the controller's kinematic model (arms + torso joints).
Links that move with joints outside that model (fingers, neck, legs) are swept over their joint ranges.
Each part is written both as a convex polytope (the planes of its convex hull) and, for the arm parts,
as a set of spheres covering it. The controller keeps every listed sphere set away from every listed
polytope: the distance of a sphere center from a polytope is the largest signed plane distance, which is
never more than the true distance (conservative).

File format (lengths in m, coordinates in the part frame):
  polytope <name> <frame> <n>        then n lines "nx ny nz off"   (inside: n.x + off <= 0)
  spheres  <name> <frame> <n>        then n lines "x y z r"
  pair     <spheres name> <polytope name>
"""

import itertools
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull

try:
    import coal
except ImportError:
    import hppfcl as coal

FINGERS = ("thumb_add", "thumb_prox", "thumb_dist", "index_add", "index_prox", "index_dist", "middle_prox",
           "middle_dist", "ring_prox", "ring_dist", "pinkie_prox", "pinkie_dist")
FINGER_LINKS = ("hand_index_1", "hand_index_2", "hand_index_3", "hand_middle_1", "hand_middle_2", "hand_ring_1",
                "hand_ring_2", "hand_pinkie_1", "hand_pinkie_2", "hand_thumb_1", "hand_thumb_2", "hand_thumb_3")
# Leg sweep while walking (rad): hip pitch, hip roll, hip yaw, knee.
LEG_SWEEP = {"hip_pitch": (-0.35, 0.45), "hip_roll": (-0.05, 0.2), "hip_yaw": (-0.2, 0.2), "knee": (-0.6, 0.0)}


def parts(s: str, walking: bool):
    """(name, frame, links, swept joints {name: (lo, hi)} or None for the URDF limits)."""
    o = "l" if s == "r" else "r"
    side = {
        "hand": (f"{s}_hand_palm", [f"{s}_hand_palm", f"{s}_wrist_2"] + [f"{s}_{n}" for n in FINGER_LINKS],
                 {f"{s}_{j}": None for j in FINGERS + ("wrist_pitch",)}),
        "forearm": (f"{s}_forearm", [f"{s}_forearm", f"{s}_wrist_1"], {f"{s}_wrist_yaw": None}),
        "upper_arm": (f"{s}_upper_arm", [f"{s}_upper_arm"], {}),
        "shoulder": (f"{s}_shoulder_3", [f"{s}_shoulder_3"], {}),
        "thigh": ("root_link", [f"{s}_hip_1", f"{s}_hip_2", f"{s}_hip_3", f"{s}_upper_leg"],
                  {f"{s}_{j}": LEG_SWEEP[j] for j in ("hip_pitch", "hip_roll", "hip_yaw")} if walking else {}),
        "shin": ("root_link", [f"{s}_lower_leg"], {f"{s}_{j}": LEG_SWEEP[j] for j in LEG_SWEEP} if walking else {}),
    }
    return {f"{s}_{k}": v for k, v in side.items()}


BODY = {
    "pelvis": ("root_link", ["root_link"], {}),
    "torso_1": ("torso_1", ["torso_1"], {}),
    "torso_2": ("torso_2", ["torso_2"], {}),
    "chest": ("chest", ["chest"], {}),
    "head": ("chest", ["neck_2", "neck_3", "head", "realsense"],
             {"neck_pitch": None, "neck_roll": None, "neck_yaw": None, "camera_tilt": None}),
}
SPHERE_RADIUS = {"hand": 0.035, "forearm": 0.04, "upper_arm": 0.045}


def shape_points(g) -> np.ndarray:
    s = g.geometry
    if isinstance(s, coal.Box):
        return np.array(list(itertools.product((-1, 1), repeat=3))) * np.asarray(s.halfSide)
    p = np.asarray(s.vertices(), dtype=float)
    return p[ConvexHull(p).vertices]


def swept_points(model, geoms, frame, links, sweep, samples=5):
    """Hull vertices of the links in `frame`, over the grid of the swept joint ranges."""
    data = model.createData()
    ranges = []
    for j, lim in sweep.items():
        i = model.joints[model.getJointId(j)].idx_q
        lo, hi = lim if lim is not None else (model.lowerPositionLimit[i], model.upperPositionLimit[i])
        ranges.append((i, np.linspace(lo, hi, samples if len(sweep) <= 4 else 3)))
    local = {n: shape_points(geoms[n]) for n in links}
    fid = model.getFrameId(frame)
    pts = []
    grids = itertools.product(*[r for _, r in ranges]) if ranges else [()]
    # hand: fingers closing together (prox = dist), thumb adduction and wrist pitch on their own
    if any("prox" in model.names[_joint(model, i)] for i, _ in ranges):
        grids = []
        for a, b, w in itertools.product(np.linspace(0, 1, 7), np.linspace(0, 1, 4), np.linspace(0, 1, 3)):
            vals = []
            for i, r in ranges:
                n = model.names[_joint(model, i)]
                x = w if "wrist" in n else b if n.endswith("thumb_add") else a
                vals.append(r[0] + x * (r[-1] - r[0]))
            grids.append(tuple(vals))
    for vals in grids:
        q = pin.neutral(model)
        for (i, _), v in zip(ranges, vals):
            q[i] = v
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        inv = data.oMf[fid].inverse()
        for n, p in local.items():
            g = geoms[n]
            M = inv * data.oMi[g.parentJoint] * g.placement
            pts.append(p @ M.rotation.T + M.translation)
    p = np.concatenate(pts)
    return p[ConvexHull(p).vertices]


def _joint(model, idx_q):
    return next(j for j in range(1, model.njoints) if model.joints[j].idx_q == idx_q)


def directions(level=2) -> np.ndarray:
    """Unit vertices of a subdivided icosahedron (12, 42, 162, ... directions)."""
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [np.array(x) / np.linalg.norm(x) for x in v]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    for _ in range(level):
        cache, nf = {}, []
        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k not in cache:
                m = v[a] + v[b]
                v.append(m / np.linalg.norm(m))
                cache[k] = len(v) - 1
            return cache[k]
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v)


DIRS = directions(2)


def planes(points) -> np.ndarray:
    """k-DOP around the points: one plane per direction, n.x + off <= 0 inside (an outer bound of the hull)."""
    return np.hstack([DIRS, -(points @ DIRS.T).max(0)[:, None]])


def depth(planes_, x):
    """Distance from the hull surface, positive inside."""
    return -(x @ planes_[:, :3].T + planes_[:, 3]).max(axis=-1)


def surface_samples(points, n=4000, seed=0):
    hull = ConvexHull(points)
    tri = points[hull.simplices]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    rng = np.random.default_rng(seed)
    k = rng.choice(len(tri), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = tri[k]
    s = t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])
    return np.vstack([points, s])


def sphere_cover(points, r):
    """Spheres of radius r, centers inside the hull, covering its surface; greedy, least bulge first."""
    P = planes(points)
    lo, hi = points.min(0), points.max(0)
    g = np.stack(np.meshgrid(*[np.arange(a, b + 1e-9, r / 3) for a, b in zip(lo, hi)], indexing="ij"), -1).reshape(-1, 3)
    g = g[depth(P, g) >= 0]
    target = surface_samples(points, 1500)
    covered = np.zeros(len(target), bool)
    dist = np.linalg.norm(g[:, None] - target[None], axis=2) <= r
    bulge = np.maximum(r - depth(P, g), 0)
    centers = []
    while not covered.all():
        gain = (dist & ~covered).sum(1) * np.exp(-bulge / (0.3 * r))
        k = int(gain.argmax())
        if (dist[k] & ~covered).sum() == 0:
            raise RuntimeError("cannot cover")
        centers.append(g[k])
        covered |= dist[k]
    c = np.array(centers)
    return c, float(bulge[[int(np.where((g == x).all(1))[0][0]) for x in c]].max())


def main():
    urdf, out = sys.argv[1], sys.argv[2]
    dirs = [str(p) for p in Path(urdf).resolve().parents if (p / "ergoCub").is_dir()][:1]
    model, cmodel, _ = pin.buildModelsFromUrdf(urdf, package_dirs=dirs)
    geoms = {model.frames[g.parentFrame].name: g for g in cmodel.geometryObjects}
    allparts = dict(BODY)
    for s in "rl":
        allparts.update(parts(s, "--walking" in sys.argv))
    lines, hulls = [], {}
    for name, (frame, links, sweep) in allparts.items():
        pts = swept_points(model, geoms, frame, links, sweep)
        hulls[name] = pts
        P = planes(pts)
        lines.append(f"polytope {name} {frame} {len(P)}")
        lines += [" ".join(f"{x:.5f}" for x in e) for e in P]
        print(f"{name:12s} {frame:14s} planes {len(P):4d}  size {np.round(pts.max(0) - pts.min(0), 3)}")
    for s in "rl":
        for k, r in SPHERE_RADIUS.items():
            name = f"{s}_{k}"
            c, b = sphere_cover(hulls[name], r)
            lines.append(f"spheres {name} {allparts[name][0]} {len(c)}")
            lines += [f"{x[0]:.5f} {x[1]:.5f} {x[2]:.5f} {r:.4f}" for x in c]
            print(f"{name:12s} spheres {len(c):3d}  r {r}  max bulge {b:.3f}")
    body = ["pelvis", "torso_1", "torso_2", "chest", "head", "r_thigh", "l_thigh", "r_shin", "l_shin"]
    pairs = []
    for s, o in (("r", "l"), ("l", "r")):
        for k in ("hand", "forearm"):
            pairs += [(f"{s}_{k}", b) for b in body]
        pairs += [(f"{s}_upper_arm", b) for b in ("pelvis", f"{s}_thigh", "head")]
        if s == "r":  # arm-arm, once per couple of parts
            pairs += [(f"r_{a}", f"l_{b}") for a in ("hand", "forearm", "upper_arm")
                      for b in ("hand", "forearm", "upper_arm", "shoulder")]
            pairs += [("l_hand", "r_shoulder"), ("l_forearm", "r_shoulder")]
    pairs = [p for p in pairs if not (p[0].endswith("upper_arm") and p[1].endswith("upper_arm"))]
    lines += [f"pair {a} {b}" for a, b in pairs]
    Path(out).write_text("\n".join(lines) + "\n")
    print(f"{len(pairs)} pairs -> {out}")


if __name__ == "__main__":
    main()
