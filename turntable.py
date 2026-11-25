"""Turntable reconstruction from a recording (rec-XXXX folder written by oak_scan_app.py).
Motor turntable = constant speed, so each frame's angle is time / revolution period. From the recording:
  1. object pixels by color (anything not platter-colored over the platter)
  2. period: when the object's image returns to frame 0
  3. axis position + spin direction: the one that makes nearby views line up
  4. fuse one+ revolutions into a TSDF mesh.
python turntable.py rec-XXXX  -> rec-XXXX/model.ply + model.stl"""
import glob, json, os, sys
import cv2, numpy as np, open3d as o3d
from scipy.spatial import cKDTree

CHROMA = 9          # Lab color distance from the platter that counts as object (lower = grabs more)
RADIUS = 0.06       # m from the axis kept as object (drops background); raise for bigger objects
SEARCH = 0.06       # m around the first guess searched for the axis
COLOR_W = 0.02      # m of distance a full color difference is worth when matching views


def rays(K, W, H):
    return ((np.arange(W)[None, :] - K[0][2]) / K[0][0] * np.ones((H, 1)),
            (np.arange(H)[:, None] - K[1][2]) / K[1][1] * np.ones((1, W)))


def platter(color):
    """Outline of the bright platter + mask of bright pixels."""
    g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    _, bright = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cs, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return max(cs, key=cv2.contourArea), bright > 0


def object_mask(color):
    """Pixels over/above the platter whose color differs from it.
    ponytail: needs object color != platter color; white object on white platter won't separate."""
    disc, bright = platter(color)
    region = np.zeros(color.shape[:2], np.uint8)
    cv2.drawContours(region, [cv2.convexHull(disc)], -1, 255, -1)
    x, y, w, h = cv2.boundingRect(disc)
    region[:y + h // 2, x:x + w] = 255  # tall objects poke above the disc outline; the 3D radius cut drops background
    lab = cv2.cvtColor(color, cv2.COLOR_BGR2LAB).astype(np.float32)
    plat = np.median(lab[bright & (region > 0)], axis=0)
    m = ((np.linalg.norm(lab[..., 1:] - plat[1:], axis=-1) > CHROMA) & (region > 0)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.erode(m, np.ones((5, 5), np.uint8)) > 0  # edges have smeared depth


def refine_normal(color, plane, K):
    """Tilt the platter normal until the platter outline projects to a circle (depth on white is too noisy)."""
    from scipy.optimize import minimize
    ct = platter(color)[0][:, 0]
    H, W = color.shape[:2]
    ct = ct[(ct[:, 0] > 2) & (ct[:, 0] < W - 3) & (ct[:, 1] > 2) & (ct[:, 1] < H - 3)]
    r = np.stack([(ct[:, 0] - K[0][2]) / K[0][0], (ct[:, 1] - K[1][2]) / K[1][1], np.ones(len(ct))], 1)
    sph = lambda a: np.array([np.sin(a[0]) * np.cos(a[1]), np.sin(a[0]) * np.sin(a[1]), np.cos(a[0])])

    def roundness(a):
        n = sph(a)
        P = r * (-plane[3] / (r @ n))[:, None]
        x = np.cross(n, [1, 0, 0.]); x /= np.linalg.norm(x); y = np.cross(n, x)
        q = np.stack([P @ x, P @ y], 1)
        s = np.linalg.lstsq(np.c_[2 * q, np.ones(len(q))], (q ** 2).sum(1), rcond=None)[0]
        rad = np.sqrt(s[2] + s[0] ** 2 + s[1] ** 2)
        return (np.linalg.norm(q - s[:2], axis=1) - rad).std() / rad

    n0 = plane[:3]
    a0 = np.array([np.arccos(n0[2]), np.arctan2(n0[1], n0[0])])
    best = min((minimize(roundness, a0 + [da, db], method="Nelder-Mead")
                for da in (-.15, 0, .15) for db in (-.2, 0, .2)), key=lambda s: s.fun)
    n = sph(best.x)
    if np.degrees(np.arccos(min(abs(n @ n0), 1))) > 10:
        return plane  # outline fit went somewhere silly; keep the RANSAC plane
    return np.r_[n if n @ n0 > 0 else -n, plane[3]]


def object_points(depth, color, plane, center, K, radius=None):
    """Object points (camera coords), colors 0-1, and pixel mask: color mask + above platter + near axis."""
    radius = radius or RADIUS
    rx, ry = rays(K, depth.shape[1], depth.shape[0])
    z = depth / 1000.0
    p = np.stack([rx * z, ry * z, z], -1).reshape(-1, 3)
    c = color[..., ::-1].reshape(-1, 3) / 255.0
    n = plane[:3]
    q = p - center
    rad = np.linalg.norm(q - np.outer(q @ n, n), axis=1)
    k = object_mask(color).reshape(-1) & (z.reshape(-1) > 0) & (p @ n + plane[3] > -0.01) & (rad < radius)
    return p[k], c[k], k.reshape(depth.shape)


def rot(axis, center, a):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(axis * a)[0]
    T[:3, 3] = center - T[:3, :3] @ center
    return T


def thin(p, c, n=1200):
    i = np.random.default_rng(0).permutation(len(p))[:n]
    return p[i], c[i]


def find_period(colors):
    """Frames per revolution: when the object's view returns to frame 0's. Returns (period, last full-turn frame)."""
    ys, xs = np.nonzero(object_mask(colors[0]))
    if len(xs) < 50:
        raise ValueError("no object found in first frame (lower CHROMA or add light)")
    pad = 30
    y0, y1, x0, x1 = max(ys.min() - pad, 0), ys.max() + pad, max(xs.min() - pad, 0), xs.max() + pad
    crop = lambda im: cv2.resize(im[y0:y1, x0:x1], (64, 64)).astype(np.float32)
    a = crop(colors[0])
    df = np.array([np.abs(crop(im) - a).mean() for im in colors])
    first = next((i for i in range(20, len(df)) if df[i] < 0.4 * df[5:i].max()), None)
    if first is None:
        raise ValueError("object never came back to its start view: scan at least one full turn")
    m1 = first + int(np.argmin(df[first:first + 15]))
    thr = max(2 * df[m1], 0.4 * df[5:m1].max())
    period, last, k = float(m1), m1, 2
    while int(k * period) + 8 < len(df):  # refine with later turns while the table keeps going
        g = int(k * period)
        mk = g - 8 + int(np.argmin(df[g - 8:g + 8]))
        if df[mk] > thr:
            break
        period, last, k = mk / k, mk, k + 1
    return period, last


def find_axis(F, plane, guess, period):
    """Axis position (on the platter plane) + spin sign that make views ~15 deg apart line up."""
    n = plane[:3]
    x = np.cross(n, [0, 0, 1.]); x /= np.linalg.norm(x); y = np.cross(n, x)
    trees = {i: cKDTree(np.hstack([p, c * COLOR_W])) for i, (p, c) in F.items()}
    idx = sorted(F)
    gap = max(1, int(round(period / 24 / (idx[1] - idx[0]))))
    pairs = [(idx[k], idx[k + gap]) for k in range(0, len(idx) - gap, 2)]

    def cost(c, s):
        t = 0
        for i, j in pairs:
            T = rot(n, c, s * 2 * np.pi * (j - i) / period)
            q = F[j][0] @ T[:3, :3].T + T[:3, 3]
            d, _ = trees[i].query(np.hstack([q, F[j][1] * COLOR_W]), distance_upper_bound=0.01)
            t += np.minimum(d, 0.01).mean()
        return t / len(pairs)

    best = None
    grid = np.arange(-SEARCH, SEARCH + 1e-9, 0.006)
    fine = np.arange(-0.006, 0.0061, 0.0015)
    for s in (1, -1):
        _, a, b = min((cost(guess + a * x + b * y, s), a, b) for a in grid for b in grid)
        cand = min((cost(guess + (a + u) * x + (b + v) * y, s), a + u, b + v) for u in fine for v in fine)
        if best is None or cand[0] < best[0]:
            best = (cand[0], s, guess + cand[1] * x + cand[2] * y)
    return best[2], best[1], best[0]


def rebuild(rec, log=print):
    meta = json.load(open(os.path.join(rec, "meta.json")))
    K, W, H, voxel = np.array(meta["K"]), meta["W"], meta["H"], meta["voxel"]
    if meta.get("floor") is None:
        raise ValueError("no platter plane in recording: keep 'Remove platter' on and the platter in view")
    files = sorted(glob.glob(os.path.join(rec, "*_c.jpg")))
    colors = [cv2.imread(f) for f in files]
    tfile = os.path.join(rec, "times.txt")
    t = np.loadtxt(tfile) if os.path.exists(tfile) else np.arange(len(files)) / 22.0  # old recordings: no times
    t = t - t[0]

    log("finding revolution period...")
    period, last = find_period(colors)
    T = np.interp(period, np.arange(len(t)), t)  # seconds per turn
    log(f"one turn = {period:.0f} frames ({T:.1f} s), using frames 0-{last}; finding axis...")

    plane = refine_normal(colors[0], np.array(meta["floor"]), K)
    depth = lambda i: cv2.imread(files[i].replace("_c.jpg", "_d.png"), cv2.IMREAD_UNCHANGED)
    guess = np.array(meta.get("center") or [0, 0, 0.3])
    guess = guess - (guess @ plane[:3] + plane[3]) * plane[:3]  # onto the platter plane
    step = max(1, int(period / 72))
    F = {i: thin(*object_points(depth(i), colors[i], plane, guess, K, radius=0.15)[:2])
         for i in range(0, min(int(period) + 1, len(files)), step)}
    center, sign, err = find_axis(F, plane, guess, period)
    log(f"axis found (match error {err * 1000:.1f} mm); fusing {last + 1} frames...")

    intr = o3d.camera.PinholeCameraIntrinsic(W, H, K[0][0], K[1][1], K[0][2], K[1][2])
    L = 2.2 * RADIUS + 0.04
    vol = o3d.pipelines.integration.UniformTSDFVolume(
        length=L, resolution=int(L / voxel), sdf_trunc=voxel * 4,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        origin=(center + plane[:3] * (L / 2 - 0.02) - L / 2).reshape(3, 1))
    for i in range(last + 1):
        _, _, keep = object_points(depth(i), colors[i], plane, center, K)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(colors[i][..., ::-1])),
            o3d.geometry.Image(np.where(keep, depth(i), 0).astype(np.uint16)),
            depth_scale=1000.0, depth_trunc=2.0, convert_rgb_to_intensity=False)
        vol.integrate(rgbd, intr, np.linalg.inv(rot(plane[:3], center, sign * 2 * np.pi * t[i] / T)))
    mesh = vol.extract_triangle_mesh()
    if len(mesh.triangles):
        ids, cnt, _ = mesh.cluster_connected_triangles()
        mesh.remove_triangles_by_mask(np.asarray(ids) != int(np.argmax(cnt)))  # keep the biggest piece
        mesh.remove_unreferenced_vertices()
    # platter center -> origin, platter normal -> +Y, so it sits upright in viewers/slicers
    up = plane[:3]
    x = np.cross(up, [0, 0, 1.]); x /= np.linalg.norm(x); z = np.cross(x, up)
    M = np.eye(4); M[:3, :3] = np.stack([x, up, z]); M[:3, 3] = -M[:3, :3] @ center
    mesh.transform(M)
    mesh.compute_vertex_normals()
    for ext in ("ply", "stl"):
        o3d.io.write_triangle_mesh(os.path.join(rec, "model." + ext), mesh)
    log(f"done: {len(mesh.triangles)} triangles, saved {os.path.basename(rec.rstrip("/\\"))}/model.ply + .stl")
    return mesh


if __name__ == "__main__":
    ax, ce = np.array([0, -1.0, 0]), np.array([0.01, 0, 0.3])
    assert np.allclose(rot(ax, ce, 0.3) @ rot(ax, ce, -0.3), np.eye(4))  # self-check
    if len(sys.argv) > 1:
        rebuild(sys.argv[1])
