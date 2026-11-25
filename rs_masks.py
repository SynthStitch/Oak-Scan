"""Write RealityScan masks (<photo>.jpg.mask.png, white = use) for a fixed-camera turntable recording.
Keeps what MOVES over the turn (platter + object), drops what's static (backdrop, stand, room):
  moving = differs from the median of all photos -> convex hull -> grown upward (smooth/round tops barely
  change as they spin). Extra plain backdrop inside the mask is harmless (no features).
python rs_masks.py rec-XXXX [--preview out.png]"""
import glob, os, sys
import cv2, numpy as np

SC = 0.25      # work scale
DIFF = 18      # 0-255 change from the median that counts as moving
UP = 0.12      # grow the mask upward by this fraction of image height (object tops)


def small(f):
    return cv2.GaussianBlur(cv2.resize(cv2.imread(f), None, fx=SC, fy=SC), (5, 5), 0).astype(np.int16)


def make_mask(s, med, full_shape):
    m = (np.abs(s - med).max(-1) > DIFF).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    pts = cv2.findNonZero(m)
    out = np.zeros_like(m)
    if pts is None:
        return cv2.resize(out, full_shape[1::-1])
    cv2.fillConvexPoly(out, cv2.convexHull(pts), 1)
    k = int(UP * out.shape[0])
    up = np.zeros_like(out)
    up[:-k] = out[k:]          # shifted copy, k rows up
    out = np.maximum(out, cv2.dilate(up, np.ones((k, 1), np.uint8), anchor=(0, 0)))
    out = cv2.dilate(out, np.ones((7, 7), np.uint8))
    return cv2.resize(out * 255, full_shape[1::-1], interpolation=cv2.INTER_NEAREST)


if __name__ == "__main__":
    fs = sorted(glob.glob(os.path.join(sys.argv[1], "photos", "*.jpg")))
    med = np.median(np.stack([small(f) for f in fs[::3]]), axis=0).astype(np.int16)
    shape = cv2.imread(fs[0]).shape
    if "--preview" in sys.argv:
        tiles = []
        for i in np.linspace(0, len(fs) - 1, 4).astype(int):
            im = cv2.imread(fs[i]); k = make_mask(small(fs[i]), med, shape)
            im[k == 0] = (im[k == 0] * 0.2).astype(np.uint8)
            tiles.append(cv2.resize(im, (400, 300)))
        cv2.imwrite(sys.argv[sys.argv.index("--preview") + 1], np.hstack(tiles))
    else:
        for f in fs:
            cv2.imwrite(f + ".mask.png", make_mask(small(f), med, shape))
        print(len(fs), "masks written")
