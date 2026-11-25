"""Turntable scan stand: reference turntable + ring cup + arm + upright + tilt head.
Edit the numbers below, then Utilities > Add-Ins > Scripts > ScanStand > Run (new doc each run).
Layout: turntable centered at origin, button opening on +X, camera arm on -X, Z up."""
import traceback
import adsk.core, adsk.fusion

# ---- turntable: measured (mm) ----
BASE_D = 145.0     # stationary bottom disc diameter
BASE_H = 30.0     # stationary disc height (table -> underside of spinning top)
PLATTER_D = 165.0  # spinning top diameter (~6.5")
PLATTER_T = 8.0    # spinning top thickness (reference only)

# ---- ring cup ----
CLR = 0.6          # radial fit clearance ring <-> base
WALL = 3.0
FLOOR = 3.0        # lip under the base edge; turntable weight holds the stand down
LIP = 10.0         # lip width inward (keep clear of the rubber feet)
RING_H = 15.0      # wall height from table; must stay below the spinning top
BTN_GAP = 46.0     # opening for the 40 mm button strip, centered on +X

# ---- arm / upright / head ----
ARM_MIN = 200.0    # closest upright position (turntable center -> upright center)
ARM_MAX = 310.0    # farthest; camera sits ~65 mm in front of the upright
ARM_AT = 260.0     # position shown in the model
NUT_W = 10.4       # channel under the arm slot: M6 hex head/nut (10 mm AF) slides but can't spin
NUT_D = 5.5        # channel depth
ARM_W = 30.0
ARM_T = 10.0
TAB_L = 25.0       # lap joint length ring tab <-> arm (2x M3 self-tapping screws)
UP_H = 200.0       # upright height
UP_W = 30.0        # upright size along X
UP_T = 10.0        # upright size along Y
SOCKET = 20.0      # depth upright sits in the arm foot
FIT = 0.2          # print fit clearance per side
BOLT = 6.6         # M6 clearance: slider clamp bolt through the upright slot
GP_T = 3.0         # GoPro prong thickness
GP_GAP = 3.2       # GoPro gap between prongs (2-prong adapter slots in here)
GP_R = 7.5         # GoPro prong end radius
GP_HOLE = 5.3      # M5 GoPro thumbscrew clearance

assert RING_H < FLOOR + BASE_H - 2, "ring wall would rub the spinning top"

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def cm(v):
    return v / 10.0


def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(cm(x), cm(y), cm(z))


def val(mm):
    return adsk.core.ValueInput.createByReal(cm(mm))


def profiles(sk):
    return [sk.profiles.item(i) for i in range(sk.profiles.count)]


def extrude(comp, prof, z0, h, op, body=None):
    inp = comp.features.extrudeFeatures.createInput(prof, op)
    inp.setDistanceExtent(False, val(h))
    if z0:
        inp.startExtent = adsk.fusion.OffsetStartDefinition.create(val(z0))
    if body:
        inp.participantBodies = [body]
    feat = comp.features.extrudeFeatures.add(inp)
    return body or feat.bodies.item(0)


def box(comp, x0, x1, y0, y1, z0, z1, op=NEW, body=None):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(x0, y0), pt(x1, y1))
    return extrude(comp, sk.profiles.item(0), z0, z1 - z0, op, body)


def disc(comp, r_out, z0, z1, op=NEW, body=None, r_in=0.0, cx=0.0, cy=0.0):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy), cm(r_out))
    if r_in:
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy), cm(r_in))
    prof = max(profiles(sk), key=lambda p: p.profileLoops.count)  # annulus if r_in
    return extrude(comp, prof, z0, z1 - z0, op, body)


def cut_y(comp, body, x0, x1, z0, z1=None):
    """Cut through everything along Y: a hole at (x0, z0) of diameter x1, or a rectangle."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    m = sk.modelToSketchSpace
    if z1 is None:
        sk.sketchCurves.sketchCircles.addByCenterRadius(m(pt(x0, 0, z0)), cm(x1 / 2))
    else:
        sk.sketchCurves.sketchLines.addTwoPointRectangle(m(pt(x0, 0, z0)), m(pt(x1, 0, z1)))
    inp = comp.features.extrudeFeatures.createInput(sk.profiles.item(0), CUT)
    inp.setSymmetricExtent(val(500), True)
    inp.participantBodies = [body]
    comp.features.extrudeFeatures.add(inp)


def prongs(comp, body, x0, xh, z, zb, spans):
    """GoPro prongs, one per (y0, y1) span: flat from x0 to the hole center xh, rounded end radius
    GP_R, solid down to the bed at zb behind the hole and a 45 deg chamfer under the round end,
    so the part prints standing on its bottom edge without supports."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    m = sk.modelToSketchSpace
    t = GP_R * 0.7071  # 45 deg tangent point on the round end
    poly = [(x0, zb), (xh, zb), (xh, z - 2 * t), (xh + t, z - t), (xh, z), (xh, z + GP_R), (x0, z + GP_R)]
    lines = sk.sketchCurves.sketchLines
    for (a, b), (c, d) in zip(poly, poly[1:] + poly[:1]):
        lines.addByTwoPoints(m(pt(a, 0, b)), m(pt(c, 0, d)))
    sk.sketchCurves.sketchCircles.addByCenterRadius(m(pt(xh, 0, z)), cm(GP_R))
    profs = adsk.core.ObjectCollection.create()
    for p in profiles(sk):
        profs.add(p)
    # XZ sketch normal can point -Y: extrude both sides of a centered window instead of trusting the sign
    inp = comp.features.extrudeFeatures.createInput(profs, JOIN)
    inp.setSymmetricExtent(val(500), True)
    inp.participantBodies = [body]
    feat = comp.features.extrudeFeatures.add(inp)
    # cut away everything between/outside the spans (only in front of x0, so the bridge is untouched)
    edges = [-250] + [y for s in sorted(spans) for y in s] + [250]
    for a, b in zip(edges[::2], edges[1::2]):
        box(comp, x0, xh + GP_R + 1, a, b, zb - 1, z + GP_R + 1, CUT, body)
    return feat


def component(root, name):
    comp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    comp.name = name
    return comp


def build(root):
    r_in = BASE_D / 2 + CLR
    r_out = r_in + WALL
    top = FLOOR + BASE_H  # underside of spinning top

    # reference turntable (not printed)
    ref = component(root, "Turntable (ref)")
    disc(ref, BASE_D / 2, FLOOR, top).name = "Base (stationary)"
    disc(ref, PLATTER_D / 2, top, top + PLATTER_T).name = "Platter (spins)"

    # ring cup: lip floor + wall + tab toward the arm, open on +X for buttons
    rc = component(root, "Ring")
    ring = disc(rc, r_out, 0, FLOOR, r_in=BASE_D / 2 - LIP)
    disc(rc, r_out, 0, RING_H, JOIN, ring, r_in=r_in)
    tab_z = ARM_T / 2
    box(rc, -(r_out + TAB_L), -(r_in + 0.5), -ARM_W / 2, ARM_W / 2, tab_z, ARM_T, JOIN, ring)
    box(rc, BASE_D / 2 - LIP - 5, r_out + 5, -BTN_GAP / 2, BTN_GAP / 2, 0, RING_H, CUT, ring)
    screws = [-(r_out + 7), -(r_out + TAB_L - 7)]
    for x in screws:
        disc(rc, 3.4 / 2, tab_z, ARM_T, CUT, ring, cx=x)
    ring.name = "Ring"

    # arm: lies on the table, steps under the ring tab; slot + bolt-head channel let the foot
    # slide ARM_MIN..ARM_MAX. Channel is open at the far end to slide the 2 bolts in.
    ac = component(root, "Arm")
    x_end = -(ARM_MAX + 40)
    arm = box(ac, x_end, -(r_out + TAB_L), -ARM_W / 2, ARM_W / 2, 0, ARM_T)
    box(ac, -(r_out + TAB_L), -(r_out + 0.5), -ARM_W / 2, ARM_W / 2, 0, tab_z, JOIN, arm)
    s0, s1 = -(ARM_MAX + 25 + BOLT / 2), -(ARM_MIN - 25 - BOLT / 2)
    box(ac, s0, s1, -BOLT / 2, BOLT / 2, 0, ARM_T, CUT, arm)
    box(ac, x_end - 1, s1, -NUT_W / 2, NUT_W / 2, 0, NUT_D, CUT, arm)
    for x in screws:
        disc(ac, 2.5 / 2, 0, tab_z, CUT, arm, cx=x)  # pilot holes for M3 self-tappers
    arm.name = "Arm"

    # foot: sits on the arm, 2x M6 (heads in the channel, wing nuts on top), socket for upright
    xc = -ARM_AT
    fc = component(root, "Foot")
    foot_top = ARM_T + SOCKET + 5
    foot = box(fc, xc - 35, xc + 35, -30, 30, ARM_T, foot_top)
    box(fc, xc - UP_W / 2 - FIT, xc + UP_W / 2 + FIT, -UP_T / 2 - FIT, UP_T / 2 + FIT,
        ARM_T + 5, foot_top, CUT, foot)
    for dx in (-25, 25):
        disc(fc, BOLT / 2, ARM_T, foot_top, CUT, foot, cx=xc + dx)
    foot.name = "Foot"

    # upright with a slot for the slider's clamp bolt (height)
    uc = component(root, "Upright")
    zu = ARM_T + 5
    up = box(uc, xc - UP_W / 2, xc + UP_W / 2, -UP_T / 2, UP_T / 2, zu, zu + UP_H)
    cut_y(uc, up, xc - BOLT / 2, xc + BOLT / 2, foot_top + 15, zu + UP_H - 15)
    up.name = "Upright"

    # GoPro slider: plate beside the upright (M6 bolt through the slot sets height), keyed rib
    # stops it spinning, bridge across the upright's front puts 3 prongs centered on the turntable
    hc = component(root, "GoPro slider")
    zp = zu + UP_H - 30  # shown near the top of the slot
    y0 = UP_T / 2 + 0.5  # 0.5 mm sliding gap to the upright side
    xu = xc + UP_W / 2  # upright front face
    # prints standing on its bottom edge (z = zb): bridge and prongs reach the bed -> no supports
    top = y0 + 6
    w = 3 * GP_T + 2 * GP_GAP
    zb = zp - 20
    head = box(hc, xc - UP_W / 2, xu + 5.5, y0, top, zb, zp + 20)
    box(hc, xu + 0.5, xu + 5.5, -w / 2 - 2, top, zb, zp + GP_R + 2, JOIN, head)
    box(hc, xc - BOLT / 2 + 0.2, xc + BOLT / 2 - 0.2, UP_T / 2 - 2.5, y0,
        zp - 15, zp + 15, JOIN, head)  # key rib rides in the slot
    cut_y(hc, head, xc, BOLT, zp)
    x0, xh = xu + 5.5, xu + 5.5 + 8
    spans = [(-w / 2 + i * (GP_T + GP_GAP), -w / 2 + i * (GP_T + GP_GAP) + GP_T) for i in range(3)]
    prongs(hc, head, x0, xh, zp, zb, spans)
    cut_y(hc, head, xh, GP_HOLE, zp)
    head.name = "GoPro slider"

    # OAK-D Lite on the 2-prong adapter, approx position at 0 deg tilt (lenses facing +X)
    cc = component(root, "OAK-D Lite (ref)")
    box(cc, xh + 21, xh + 38.5, -45.5, 45.5, zp - 14, zp + 14).name = "OAK-D Lite"


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        build(design.rootComponent)
        app.activeViewport.fit()
    except Exception:
        if ui:
            ui.messageBox("ScanStand failed:\n{}".format(traceback.format_exc()))
