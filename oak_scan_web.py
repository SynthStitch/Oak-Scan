"""OAK-D Lite turntable scanner, web UI.  python oak_scan_web.py  -> http://127.0.0.1:8765
Record (13MP photos + depth) -> Build: masks + RealityScan photogrammetry -> model shown in the page.
Recordings + models: rec-<time>/ next to this file."""
import json, os, subprocess, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import cv2, numpy as np, open3d as o3d, depthai as dai
import turntable as tt

W, H = 640, 480
PHOTO_FPS = 8      # 13MP photos/s while recording; also caps the whole camera pipeline
PORT = 8765
MAX_REC_S = 120    # auto-stop recording (2 turns take ~40 s; a forgotten recording writes ~30 MB/s)
OUT = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(OUT, "web")
RS = r"C:\Program Files\Epic Games\RealityScan_2.2\RealityScan.exe"


def fit_floor(depth, intr, max_d):
    """RANSAC the platter plane (camera coords), normal pointing at the camera."""
    pc = o3d.geometry.PointCloud.create_from_depth_image(
        o3d.geometry.Image(depth), intr, depth_scale=1000.0, depth_trunc=max_d).voxel_down_sample(0.004)
    if len(pc.points) < 100:
        return None
    plane, inl = pc.segment_plane(0.004, 3, 300)
    if len(inl) < 0.15 * len(pc.points):
        return None
    return -np.array(plane) if plane[3] < 0 else np.array(plane)


class Scanner:
    def __init__(self):
        self.s = {"min_d": 15, "max_d": 45, "focus": 160, "chroma": tt.CHROMA, "photos": True}  # UI settings
        self.recording = self.building = False
        self.running, self.focus_dirty, self.af_request, self.sweep_pos = True, False, False, None
        self.floor, self.rec, self.frames, self.n_photos, self.fps, self.dist = None, None, 0, 0, 0.0, 0.0
        self.msg, self.model = "starting camera...", None
        recs = sorted(d for d in os.listdir(OUT) if d.startswith("rec-"))
        self.last_rec = os.path.join(OUT, recs[-1]) if recs else None
        self.jpg, self.jpg_t = None, 0.0  # latest color | depth side-by-side frame, fetched one at a time
        self.pts = np.zeros(0, np.float32)
        threading.Thread(target=self.worker, daemon=True).start()

    # --- commands from the page ---
    def record(self):
        if self.recording:
            self.recording = False
            self.msg = f"recorded {self.frames} frames + {self.n_photos} photos - press Build"
        elif self.floor is None:
            self.msg = "platter not found - check Min/Max depth"
        else:
            self.frames, self.n_photos, self.rec, self.recording = 0, 0, None, True
            self.rec_t0 = time.time()

    def build(self):
        if self.building or self.recording or not self.last_rec:
            self.msg = "record a scan first" if not self.last_rec else self.msg
            return
        self.building = True
        threading.Thread(target=self._build, daemon=True).start()

    def _build(self):
        rec = self.last_rec
        try:
            if os.path.isdir(os.path.join(rec, "photos")):
                self.msg = "making masks..."
                subprocess.run(["python", os.path.join(OUT, "rs_masks.py"), rec], check=True, cwd=OUT)
                self.msg = "RealityScan: aligning + meshing (minutes)..."
                photos, obj = os.path.join(rec, "photos"), os.path.join(rec, "rs_model.obj")
                subprocess.run([RS, "-newScene", "-addFolder", photos, "-align", "-selectMaximalComponent",
                                "-setReconstructionRegionAuto", "-calculateNormalModel",
                                "-selectLargestModelComponent", "-invertTrianglesSelection", "-removeSelectedTriangles",
                                "-simplify", "60000", "-exportSelectedModel", obj,
                                "-save", os.path.join(rec, "rs_project.rsproj"), "-quit"])
                if not os.path.exists(obj):
                    raise RuntimeError("RealityScan made no model (alignment failed? see its window)")
                self.model = obj
            else:
                tt.rebuild(rec, lambda s: setattr(self, "msg", s))  # older depth-only recordings
                self.model = os.path.join(rec, "model.ply")
            self.msg = f"model ready: {os.path.relpath(self.model, OUT)}"
        except Exception as e:
            self.msg = f"build failed: {e}"
        self.building = False

    def state(self):
        st = ("RECORDING" if self.recording else "BUILDING" if self.building else
              "READY" if self.floor is not None else "NO PLATTER")
        return {"state": st, "fps": round(self.fps), "dist": round(self.dist), "frames": self.frames,
                "photos": self.n_photos, "msg": self.msg, "settings": self.s,
                "model": f"/model?t={os.path.getmtime(self.model):.0f}" if self.model else None,
                "model_ext": os.path.splitext(self.model)[1][1:] if self.model else None}

    def set(self, k, v):
        if k in self.s:
            self.s[k] = type(self.s[k])(v)
            if k == "focus":
                self.focus_dirty = True
            if k == "chroma":
                tt.CHROMA = int(v)

    # --- camera thread ---
    def worker(self):
        while self.running:  # camera drops off USB sometimes: reconnect instead of dying
            try:
                self.loop()
            except Exception as e:
                self.recording = False
                self.msg = f"camera lost ({e}) - reconnecting..."
                time.sleep(3)

    def loop(self):
        with dai.Pipeline() as p:
            rgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            rgb.initialControl.setManualFocus(self.s["focus"])
            ctrl = rgb.inputControl.createInputQueue()
            left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = p.create(dai.node.StereoDepth)
            left.requestOutput((W, H)).link(stereo.left)
            right.requestOutput((W, H)).link(stereo.right)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            stereo.setExtendedDisparity(True)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(W, H)
            sync = p.create(dai.node.Sync)
            rgb.requestOutput((W, H), dai.ImgFrame.Type.BGR888i).link(sync.inputs["rgb"])
            stereo.depth.link(sync.inputs["depth"])
            q = sync.out.createOutputQueue(maxSize=1, blocking=False)
            full = rgb.requestFullResolutionOutput(dai.ImgFrame.Type.NV12, fps=PHOTO_FPS)
            enc = p.create(dai.node.VideoEncoder).build(
                full, frameRate=PHOTO_FPS, profile=dai.VideoEncoderProperties.Profile.MJPEG, quality=95)
            jq = enc.out.createOutputQueue(maxSize=8, blocking=False)
            K = np.array(p.getDefaultDevice().readCalibration()
                         .getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, W, H))
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
            rx, ry = tt.rays(K, W, H)
            p.start()
            self.msg, t0, n = "camera ready", time.time(), 0
            while self.running and p.isRunning():
                g = q.get()
                color = g["rgb"].getCvFrame()
                if self.focus_dirty:
                    c = dai.CameraControl()
                    c.setManualFocus(self.s["focus"])
                    ctrl.send(c)
                    self.focus_dirty = False
                self.focus_sweep(color, ctrl)
                depth = g["depth"].getFrame().copy()
                depth[(depth < self.s["min_d"] * 10) | (depth > self.s["max_d"] * 10)] = 0
                try:
                    obj = tt.object_mask(color) & (depth > 0)
                except ValueError:  # no bright platter in view (e.g. speckle disc): show all in-range depth
                    obj = depth > 0
                if not self.recording and n % 10 == 0:
                    plat = depth.copy()
                    plat[obj] = 0
                    self.floor = fit_floor(plat, intr, self.s["max_d"] / 100)
                if self.recording:
                    self.save_frame(color, depth, obj, g, K, rx, ry)
                    if time.time() - self.rec_t0 > MAX_REC_S:
                        self.record()  # stop
                        self.msg = f"auto-stopped after {MAX_REC_S} s - " + self.msg
                for m in jq.tryGetAll():  # full-res JPEGs encoded on the camera
                    if self.recording and self.rec and self.s["photos"]:
                        os.makedirs(os.path.join(self.rec, "photos"), exist_ok=True)
                        with open(os.path.join(self.rec, "photos", f"{self.n_photos:05d}.jpg"), "wb") as f:
                            f.write(bytes(m.getData()))
                        self.n_photos += 1
                n += 1
                if n % 30 == 0:
                    self.fps, t0 = 30 / (time.time() - t0), time.time()
                self.dist = float(np.median(depth[obj]) / 10) if obj.any() else 0.0
                self.publish(color, depth, obj, rx, ry, n)

    def focus_sweep(self, color, ctrl):
        """Step the lens, score sharpness on the object, lock the best (the camera's AF trigger didn't move it)."""
        if self.af_request:
            self.af_request, self.sweep, self.sweep_i, self.sweep_wait = False, [], 0, 0
            self.sweep_pos = list(range(40, 256, 12))
        if not self.sweep_pos:
            return
        if self.sweep_wait == 0:
            c = dai.CameraControl()
            c.setManualFocus(self.sweep_pos[self.sweep_i])
            ctrl.send(c)
            self.sweep_wait = 3  # let the lens settle
            return
        self.sweep_wait -= 1
        if self.sweep_wait:
            return
        roi = color[H // 4:3 * H // 4, W // 4:3 * W // 4]
        self.sweep.append(cv2.Laplacian(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
        self.sweep_i += 1
        self.msg = f"focusing {self.sweep_i}/{len(self.sweep_pos)}..."
        if self.sweep_i == len(self.sweep_pos):
            self.s["focus"] = self.sweep_pos[int(np.argmax(self.sweep))]
            self.sweep_pos, self.focus_dirty = None, True
            self.msg = f"focus locked at {self.s['focus']}"

    def save_frame(self, color, depth, obj, g, K, rx, ry):
        if self.rec is None:
            z = depth[obj] / 1000.0
            c = [float(np.median(rx[obj] * z)), float(np.median(ry[obj] * z)), float(np.median(z))] \
                if obj.sum() > 100 else None
            self.rec = self.last_rec = os.path.join(OUT, "rec-" + time.strftime("%Y%m%d-%H%M%S"))
            os.makedirs(self.rec)
            json.dump({"K": K.tolist(), "W": W, "H": H, "floor": self.floor.tolist(), "center": c,
                       "voxel": 0.002}, open(os.path.join(self.rec, "meta.json"), "w"))
            self.times = open(os.path.join(self.rec, "times.txt"), "w")
        cv2.imwrite(os.path.join(self.rec, f"{self.frames:05d}_c.jpg"), color)
        cv2.imwrite(os.path.join(self.rec, f"{self.frames:05d}_d.png"), depth)
        self.times.write(f"{g['rgb'].getTimestamp().total_seconds():.4f}\n")
        self.times.flush()
        self.frames += 1

    def publish(self, color, depth, obj, rx, ry, n):
        view = color.copy()
        view[obj] = (view[obj] * 0.55 + np.array([255, 200, 0]) * 0.45).astype(np.uint8)  # aqua tint = object
        span = max(self.s["max_d"] - self.s["min_d"], 1) * 10
        dc = cv2.applyColorMap(np.clip((depth - self.s["min_d"] * 10) / span * 255, 0, 255).astype(np.uint8),
                               cv2.COLORMAP_TURBO)
        dc[depth == 0] = 0
        self.jpg = cv2.imencode(".jpg", np.hstack([view, dc]), [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self.jpg_t = time.time()
        if n % 3 == 0 and obj.any():  # live point cloud: x, y(up), z(toward viewer), r, g, b
            v, u = np.nonzero(obj)
            if len(u) > 15000:
                k = np.random.default_rng(n).choice(len(u), 15000, replace=False)
                v, u = v[k], u[k]
            z = depth[v, u] / 1000.0
            col = color[v, u][:, ::-1] / 255.0
            self.pts = np.c_[rx[v, u] * z, -ry[v, u] * z, -z, col].astype(np.float32).ravel()


scanner = Scanner()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            self.send(open(os.path.join(WEB, "index.html"), "rb").read(), "text/html; charset=utf-8")
        elif path == "/frame":  # one JPEG per request: no long-lived connection to stall or exhaust
            if scanner.jpg is None or time.time() - scanner.jpg_t > 3:
                return self.send(b"no frame", "text/plain", 503)
            self.send(scanner.jpg, "image/jpeg")
        elif path == "/state":
            self.send(json.dumps(scanner.state()).encode(), "application/json")
        elif path == "/points":
            self.send(scanner.pts.tobytes(), "application/octet-stream")
        elif path == "/model" and scanner.model:
            self.send(open(scanner.model, "rb").read(), "application/octet-stream")
        else:
            self.send(b"not found", "text/plain", 404)

    def do_POST(self):
        path = self.path.split("?")[0]
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        if path == "/cmd/record":
            scanner.record()
        elif path == "/cmd/build":
            scanner.build()
        elif path == "/cmd/autofocus":
            scanner.af_request = True
        elif path == "/set":
            for k, v in body.items():
                scanner.set(k, v)
        else:
            return self.send(b"not found", "text/plain", 404)
        self.send(json.dumps(scanner.state()).encode(), "application/json")


if __name__ == "__main__":
    print(f"OAK Scan UI: http://127.0.0.1:{PORT}")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
