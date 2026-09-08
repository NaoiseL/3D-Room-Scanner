"""
3D room scanner: sweep capture and reconstruction.

Drives the tilt axis via tilt_test.ino over serial, captures lidar
revolutions at each stop, and turns the result into a point cloud.

Geometry
--------
The tilt axis lies along X. The scan plane contains that axis, so a point
measured at in-plane bearing theta and range d, with the axis tilted by
beta, lands at:

    x = d * cos(theta)
    y = d * sin(theta) * cos(beta)
    z = d * sin(theta) * sin(beta)

theta is measured FROM THE TILT AXIS. Set --offset so that bearing zero
points along the pivot, otherwise the whole cloud is rotated.

Two stages, deliberately separated:

    scan   drives the hardware, saves raw (beta, theta, range) to CSV
    build  reads that CSV and produces a PLY

Scanning takes minutes. Rebuilding with different calibration should be
instant, so never re-scan just to change a parameter.

Usage:
    python scan3d.py scan --lidar COM7 --arduino COM4 --out room.csv
    python scan3d.py build --raw room.csv --offset 90 --out room.ply
    python scan3d.py selftest
"""

import argparse
import csv
import math
import sys
import time

import x2l

STEP_DEG = 0.75          # one full motor step at 2.4:1 with a 1.8 deg motor
STEPS_180 = 240          # full steps to cover the sweep


def to_3d(theta_deg, distance_mm, beta_deg, oy=0.0, oz=0.0, gamma_deg=0.0):
    """
    (bearing, range, tilt) -> (x, y, z) in metres.

    oy, oz     lidar optical centre offset from the rotation axis, metres.
               This rotates WITH the lidar, so it must be applied before
               the tilt rotation, not after.
    gamma_deg  misalignment of the scan plane relative to the tilt axis.
               Zero means the axis lies exactly in the plane.
    """
    t = math.radians(theta_deg)
    b = math.radians(beta_deg)
    g = math.radians(gamma_deg)
    d = distance_mm / 1000.0

    # Point in the lidar's own frame, including any tilt of the scan plane.
    px = d * math.cos(t) * math.cos(g)
    py = d * math.sin(t) + oy
    pz = -d * math.cos(t) * math.sin(g) + oz

    # Then rotate about the tilt axis.
    return (px,
            py * math.cos(b) - pz * math.sin(b),
            py * math.sin(b) + pz * math.cos(b))


class Axis:
    """Talks to tilt_test.ino."""

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=3.0)
        time.sleep(2.5)                 # the board resets when the port opens
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def command(self, text):
        self.ser.write((text + "\n").encode())
        self.ser.flush()

    def wait_for_status(self, timeout=10.0):
        """Read until the firmware prints its status line."""
        deadline = time.time() + timeout
        seen = []
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line:
                seen.append(line)
            if line.startswith("pos "):
                return line
        raise TimeoutError(
            "no status line from the axis. Received: %r\n"
            "If that is empty, this port is probably not the Arduino."
            % (seen[-5:],)
        )

    def step(self, full_steps=1):
        self.command("F%d" % full_steps)
        return self.wait_for_status()

    def zero(self):
        # The Z command sets the datum but does not print a status line,
        # so follow it with ? which always does.
        self.command("Z")
        time.sleep(0.1)
        self.command("?")
        self.wait_for_status()


def flush_lidar(lidar):
    """Throw away everything captured while the axis was moving."""
    lidar.ser.reset_input_buffer()
    lidar.buf.clear()


def cmd_scan(args):
    lidar = x2l.X2L(args.lidar)
    axis = Axis(args.arduino)

    print("zeroing axis at its current position")
    axis.zero()

    rows = []
    seen = 0
    scans = lidar.scans()
    t0 = time.time()

    try:
        for i in range(args.steps):
            beta = i * STEP_DEG

            flush_lidar(lidar)
            time.sleep(args.settle)

            next(scans)                              # discard the partial lap
            for _ in range(args.revs):
                for theta, dist in next(scans):
                    seen += 1
                    if args.min_range * 1000 < dist <= args.max_range * 1000:
                        rows.append((round(beta, 4),
                                     round(theta, 3),
                                     round(dist, 1)))

            if i % 10 == 0:
                elapsed = time.time() - t0
                pct = 100.0 * (i + 1) / args.steps
                print("  %5.1f deg  %3.0f%%  %7d points  %4.0f s"
                      % (beta, pct, len(rows), elapsed))

            if i < args.steps - 1:
                axis.step(1)

    except KeyboardInterrupt:
        print("\ninterrupted, saving what we have")
    finally:
        lidar.close()
        axis.close()

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["beta_deg", "theta_deg", "range_mm"])
        w.writerows(rows)

    rate = 100.0 * len(rows) / seen if seen else 0.0
    print("\n%d points from %d rays (%.0f%% returned) -> %s"
          % (len(rows), seen, rate, args.out))
    print("%.0f s, %d bad checksums"
          % (time.time() - t0, lidar.stats["bad_checksum"]))

    if rate < 60.0:
        print("\nWARNING: only %.0f%% of rays returned. The X2L reaches 8 m,"
              % rate)
        print("so a room bigger than that loses most of its walls. Scan a")
        print("smaller space, or move the scanner toward the middle.")

    print("\nnext:  verify --raw %s" % args.out)


def cmd_findaxis(args):
    """
    Recover the tilt axis bearing from a raw scan.

    A ray along the tilt axis does not move as the axis tilts, so its
    measured range stays constant across the whole sweep. Every other
    bearing sweeps a cone through the room and its range varies a lot.
    So the bearing with the least range variation is the axis.

    The two poles are 180 degrees apart, so we score each candidate by
    the variation at that bearing plus the variation at its antipode.
    """
    import statistics

    bins = {}
    with open(args.raw) as f:
        for row in csv.DictReader(f):
            b = int(float(row["theta_deg"])) % 360
            bins.setdefault(b, []).append(float(row["range_mm"]))

    spread = {}
    median = {}
    for b, vals in bins.items():
        if len(vals) < args.min_points:
            continue
        med = statistics.median(vals)
        median[b] = med
        # Median absolute deviation: robust against dropouts and outliers.
        spread[b] = statistics.median([abs(v - med) for v in vals])

    if len(spread) < 180:
        print("only %d bearings had enough returns; scan may be too sparse"
              % len(spread))
        if not spread:
            return

    scored = []
    for b in spread:
        anti = (b + 180) % 360
        if anti in spread:
            scored.append((spread[b] + spread[anti], b))
    if not scored:
        print("no antipodal pairs with enough data")
        return

    scored.sort()
    print("bearing   variation   antipode   median range")
    for total, b in scored[:8]:
        anti = (b + 180) % 360
        print("%5d %11.0f mm %7d %13.0f mm"
              % (b, total, anti, median[b] / 1000.0 * 1000))

    best = scored[0][1]
    typical = statistics.median(list(spread.values()))
    print("\nbest axis bearing: %d deg" % best)
    print("variation there is %.0f mm against %.0f mm typical"
          % (scored[0][0], 2 * typical))

    if scored[0][0] > typical:
        print("WARNING: that is not much flatter than average, so the pole")
        print("may be occluded or aimed somewhere with no stable return.")

    print("\ntry:  build --raw %s --offset %d" % (args.raw, best))


def _load_raw(path):
    import numpy as np
    beta, theta, dist = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            beta.append(float(row["beta_deg"]))
            theta.append(float(row["theta_deg"]))
            dist.append(float(row["range_mm"]))
    return (np.array(beta), np.array(theta), np.array(dist) / 1000.0)


def _transform(beta, theta, dist, offset, oy=0.0, oz=0.0, gamma=0.0):
    import numpy as np
    t = np.radians(theta - offset)
    b = np.radians(beta)
    g = math.radians(gamma)

    px = dist * np.cos(t) * math.cos(g)
    py = dist * np.sin(t) + oy
    pz = -dist * np.cos(t) * math.sin(g) + oz

    cb, sb = np.cos(b), np.sin(b)
    return px, py * cb - pz * sb, py * sb + pz * cb


def _peakiness(v, bin_m=0.10):
    """Fraction of points in the two fullest bins. Floors and ceilings spike."""
    import numpy as np
    lo, hi = np.percentile(v, [0.5, 99.5])
    if hi - lo < bin_m:
        return 0.0
    counts, _ = np.histogram(v, bins=max(4, int((hi - lo) / bin_m)),
                             range=(lo, hi))
    if counts.sum() == 0:
        return 0.0
    top = np.sort(counts)[-2:].sum()
    return float(top) / float(counts.sum())


def cmd_check(args):
    """
    Diagnose a reconstruction. A real room has a floor and a ceiling, so one
    axis should show two sharp spikes holding a large share of all points.
    If no axis does, the geometry is wrong rather than the calibration.
    """
    import numpy as np

    beta, theta, dist = _load_raw(args.raw)

    if args.sweep:
        step = max(1, len(beta) // 20000)
        b, t, d = beta[::step], theta[::step], dist[::step]
        results = []
        for off in range(0, 360, 2):
            xyz = _transform(b, t, d, off)
            results.append((max(_peakiness(v) for v in xyz), off))
        results.sort(reverse=True)
        print("offset   planar score")
        for score, off in results[:8]:
            print("%5d %13.3f" % (off, score))
        print("\nbest offset by planar structure: %d" % results[0][1])
        print("typical score elsewhere: %.3f"
              % float(np.median([r[0] for r in results])))
        return

    x, y, z = _transform(beta, theta, dist, args.offset)
    for name, v in (("x", x), ("y", y), ("z", z)):
        lo, hi = np.percentile(v, [0.5, 99.5])
        counts, edges = np.histogram(v, bins=30, range=(lo, hi))
        peak = counts.max() or 1
        print("\n%s axis   %.2f to %.2f m   peakiness %.3f"
              % (name, lo, hi, _peakiness(v)))
        for c, e in zip(counts, edges):
            if c > peak * 0.04:
                print("  %6.2f m %s %d"
                      % (e, "#" * int(40.0 * c / peak), c))


def cmd_slice(args):
    """
    Plot individual tilt positions as 2D cross-sections.

    Every slice is a plane containing the tilt axis, so all of them share
    the horizontal coordinate here. If the model is right they should all
    agree near u = 0 and each should look like a plausible cut through the
    room. If they do not, the fault is in the assembly, not the offset.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    by_beta = {}
    with open(args.raw) as f:
        for row in csv.DictReader(f):
            b = round(float(row["beta_deg"]), 2)
            by_beta.setdefault(b, []).append(
                (float(row["theta_deg"]), float(row["range_mm"]) / 1000.0))

    available = sorted(by_beta)
    if not available:
        print("no data")
        return

    fig, ax = plt.subplots(figsize=(9, 9))
    for target in args.beta:
        nearest = min(available, key=lambda b: abs(b - target))
        pts = by_beta[nearest]
        t = np.radians(np.array([p[0] for p in pts]) - args.offset)
        d = np.array([p[1] for p in pts])
        # u runs along the tilt axis, v perpendicular to it in this plane.
        ax.scatter(d * np.cos(t), d * np.sin(t), s=4,
                   label="beta %.1f deg  (%d pts)" % (nearest, len(pts)))

    ax.set_aspect("equal")
    ax.axhline(0, lw=0.5, color="0.7")
    ax.axvline(0, lw=0.5, color="0.7")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("u, along the tilt axis (m)")
    ax.set_ylabel("v, perpendicular (m)")
    ax.set_title("cross-sections, offset %.0f deg" % args.offset)
    ax.legend(loc="upper right", fontsize=8)
    plt.show()


def cmd_verify(args):
    """
    Independent check that does not care what the room looks like.

    A 180 degree sweep returns the scan plane to where it started, mirrored.
    So the last slice should be the first slice reflected about the pole:
    d_last(theta) == d_first(2*pole - theta).

    Scanning candidate poles and scoring that agreement recovers the axis
    bearing without assuming anything about walls, and the residual tells
    you whether the sweep really covered 180 degrees. A poor residual at
    every candidate means the axis lost steps or the scene moved.
    """
    import statistics

    by_beta = {}
    with open(args.raw) as f:
        for row in csv.DictReader(f):
            b = round(float(row["beta_deg"]), 2)
            th = int(float(row["theta_deg"])) % 360
            by_beta.setdefault(b, {}).setdefault(th, []).append(
                float(row["range_mm"]))

    if len(by_beta) < 2:
        print("need at least two tilt positions")
        return

    betas = sorted(by_beta)
    first = {t: statistics.median(v) for t, v in by_beta[betas[0]].items()}
    last = {t: statistics.median(v) for t, v in by_beta[betas[-1]].items()}
    print("comparing beta %.2f with beta %.2f (span %.2f deg)"
          % (betas[0], betas[-1], betas[-1] - betas[0]))

    scored = []
    for pole in range(360):
        resid, n = [], 0
        for th, d in first.items():
            mirror = (2 * pole - th) % 360
            if mirror in last:
                resid.append(abs(d - last[mirror]))
                n += 1
        if n >= args.min_overlap:
            scored.append((statistics.median(resid), pole, n))

    if not scored:
        print("not enough overlapping bearings to compare")
        return

    scored.sort()
    print("\npole   median disagreement   bearings")
    for r, pole, n in scored[:6]:
        print("%4d %17.0f mm %10d" % (pole, r, n))

    best_r, best_pole, _ = scored[0]
    typical = statistics.median([s[0] for s in scored])
    print("\nbest pole: %d deg, disagreement %.0f mm against %.0f mm typical"
          % (best_pole, best_r, typical))

    if best_r > 300:
        print("\nThat is a poor match at every candidate. Likely causes:")
        print("  - the axis lost steps, so the sweep was not really 180 deg")
        print("  - something in the scene moved between the two ends")
        print("  - too few returns, so the two slices barely overlap")
    else:
        print("\nSweep is self-consistent. Use:  build --offset %d" % best_pole)


def _fit_plane_ransac(P, thresh=0.03, iters=300, seed=0):
    """Return (normal, offset, inlier_mask) for the dominant plane in P."""
    import numpy as np
    rng = np.random.default_rng(seed)
    best_mask, best_count = None, 0

    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        a, b, c = P[idx]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        mask = np.abs((P - a) @ n) < thresh
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask

    if best_mask is None or best_count < 3:
        return None, None, None

    # Least-squares refit on the inliers, which is far more accurate than
    # the three random points that found them.
    Q = P[best_mask]
    centroid = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - centroid, full_matrices=False)
    n = vt[2]
    mask = np.abs((P - centroid) @ n) < thresh
    return n, float(centroid @ n), mask


def cmd_measure(args):
    """
    Pull the dominant planes out of a scan and measure between them.

    Each wall is fitted through tens of thousands of points, so the plane
    sits far more precisely than any individual measurement does. The rms
    residual reported per plane is a direct estimate of the sensor noise
    at that range, and the spacing between parallel planes is the room
    dimension to check against a tape measure.
    """
    import numpy as np

    beta, theta, dist = _load_raw(args.raw)
    x, y, z = _transform(beta, theta, dist, args.offset,
                         args.oy, args.oz, args.gamma)
    P = np.column_stack([x, y, z])

    keep = dist > args.min_range
    P = P[keep]
    print("%d points, ignoring returns closer than %.2f m\n"
          % (len(P), args.min_range))

    planes = []
    remaining = P
    for i in range(args.planes):
        if len(remaining) < 500:
            break
        n, d, mask = _fit_plane_ransac(remaining, args.thresh, seed=i)
        if n is None or mask.sum() < args.min_inliers:
            break
        inliers = remaining[mask]
        resid = (inliers @ n - d) * 1000.0
        planes.append({
            "n": n, "d": d, "count": int(mask.sum()),
            "rms": float(np.sqrt((resid ** 2).mean())),
            "range": float(np.linalg.norm(inliers, axis=1).mean()),
        })
        remaining = remaining[~mask]

    if not planes:
        print("no planes found; try a larger --thresh")
        return

    print("plane  points   rms      mean range   normal")
    for i, p in enumerate(planes):
        print("%4d %8d %7.1f mm %9.2f m    (%+.2f %+.2f %+.2f)"
              % (i, p["count"], p["rms"], p["range"], *p["n"]))

    print("\nparallel pairs (room dimensions):")
    found = False
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            dot = abs(float(planes[i]["n"] @ planes[j]["n"]))
            if dot > 0.98:
                sep = abs(planes[i]["d"] - planes[j]["d"] * np.sign(
                    planes[i]["n"] @ planes[j]["n"]))
                print("  planes %d and %d:  %.3f m apart" % (i, j, sep))
                found = True
    if not found:
        print("  none found; increase --planes")

    print("\nperpendicular pairs (squareness):")
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            dot = abs(float(planes[i]["n"] @ planes[j]["n"]))
            if dot < 0.2:
                ang = math.degrees(math.acos(min(1.0, dot)))
                print("  planes %d and %d:  %.2f deg  (error %+.2f)"
                      % (i, j, ang, ang - 90.0))

    best = min(planes, key=lambda p: p["rms"])
    print("\nFlattest surface: %.1f mm rms over %d points at %.2f m."
          % (best["rms"], best["count"], best["range"]))
    print("Compare that against the single-point noise you measured with")
    print("x2l.py ruler at the same distance. Averaging over a plane is")
    print("what buys the accuracy back.")


def _plane_normals(P, groups):
    """Refit a plane to each fixed inlier group. Returns normals and rms."""
    import numpy as np
    out = []
    for idx in groups:
        Q = P[idx]
        c = Q.mean(axis=0)
        _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
        n = vt[2]
        resid = (Q - c) @ n
        out.append((n, float(np.sqrt((resid ** 2).mean())) * 1000.0))
    return out


def _squareness_cost(normals):
    """Penalise pairs that should be square but are not, plus plane roughness."""
    cost = 0.0
    pairs = 0
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            dot = abs(float(normals[i][0] @ normals[j][0]))
            if dot < 0.35:                      # a pair meant to be square
                cost += (dot * 90.0) ** 2       # roughly degrees off square
                pairs += 1
    if not pairs:
        return float("inf")
    rough = sum(n[1] for n in normals) / len(normals)
    return cost / pairs + rough


def cmd_calibrate(args):
    """
    Solve the extrinsic parameters from the scan itself.

    Walls, floors and ceilings are square to each other in the real world.
    If the reconstruction says otherwise, the error is in the mechanical
    model, so we search the lever arm and scan plane misalignment for the
    values that make the surfaces most nearly perpendicular.

    Plane memberships are fixed once at nominal, then those same surfaces
    are refitted for every candidate, which keeps the search fast and stops
    the assignment drifting to chase the objective.
    """
    import numpy as np

    beta, theta, dist = _load_raw(args.raw)
    keep = dist > args.min_range
    beta, theta, dist = beta[keep], theta[keep], dist[keep]

    step = max(1, len(beta) // args.sample)
    beta, theta, dist = beta[::step], theta[::step], dist[::step]
    print("calibrating on %d points" % len(beta))

    P0 = np.column_stack(_transform(beta, theta, dist, args.offset))
    groups, remaining_idx = [], np.arange(len(P0))
    for i in range(args.planes):
        sub = P0[remaining_idx]
        if len(sub) < 300:
            break
        n, d, mask = _fit_plane_ransac(sub, args.thresh, seed=i)
        if n is None or mask.sum() < args.min_inliers:
            break
        groups.append(remaining_idx[mask])
        remaining_idx = remaining_idx[~mask]

    if len(groups) < 3:
        print("found only %d planes; need at least 3 to calibrate" % len(groups))
        return
    print("using %d surfaces of sizes %s\n"
          % (len(groups), [len(g) for g in groups]))

    base = _squareness_cost(_plane_normals(P0, groups))
    best = (base, 0.0, 0.0, 0.0)

    span = args.span
    for oy in np.linspace(-span, span, args.grid):
        for oz in np.linspace(-span, span, args.grid):
            for gamma in np.linspace(-args.gamma, args.gamma, args.grid):
                P = np.column_stack(
                    _transform(beta, theta, dist, args.offset, oy, oz, gamma))
                c = _squareness_cost(_plane_normals(P, groups))
                if c < best[0]:
                    best = (c, oy, oz, gamma)

    cost, oy, oz, gamma = best
    print("uncalibrated cost   %.2f" % base)
    print("calibrated cost     %.2f" % cost)
    print("\noy     %+.3f m   lidar offset across the axis" % oy)
    print("oz     %+.3f m   lidar offset along the scan plane normal" % oz)
    print("gamma  %+.2f deg  scan plane tilt against the axis" % gamma)

    P = np.column_stack(
        _transform(beta, theta, dist, args.offset, oy, oz, gamma))
    before = _plane_normals(P0, groups)
    after = _plane_normals(P, groups)
    print("\n        squareness error, degrees")
    print("pair      before     after")
    for i in range(len(groups)):
        for j in range(i + 1, len(groups)):
            d0 = abs(float(before[i][0] @ before[j][0]))
            d1 = abs(float(after[i][0] @ after[j][0]))
            if d0 < 0.35 or d1 < 0.35:
                print("%d - %d %10.2f %9.2f"
                      % (i, j,
                         90.0 - math.degrees(math.acos(min(1.0, d0))),
                         90.0 - math.degrees(math.acos(min(1.0, d1)))))

    print("\nbuild --raw %s --offset %g --oy %.3f --oz %.3f --gamma %.2f"
          % (args.raw, args.offset, oy, oz, gamma))

    if abs(oy) > span * 0.95 or abs(oz) > span * 0.95:
        print("\nWARNING: a solution sat on the edge of the search range.")
        print("Rerun with a larger --span.")


def cmd_build(args):
    points = []

    with open(args.raw) as f:
        for row in csv.DictReader(f):
            theta = (float(row["theta_deg"]) - args.offset) % 360.0
            points.append(to_3d(theta, float(row["range_mm"]),
                                float(row["beta_deg"]),
                                args.oy, args.oz, args.gamma))

    if not points:
        print("no points in %s" % args.raw)
        return

    if args.mirror:
        # YDLIDAR bearings increase clockwise, and the tilt may drive the
        # opposite way from the model's assumption. Together those make the
        # reconstruction left-handed, so the room comes out mirrored.
        # Negating beta is the physical fix and works out to negating z.
        points = [(x, y, -z) for x, y, z in points]

    write_ply(args.out, points)

    xs = sorted(p[0] for p in points)
    ys = sorted(p[1] for p in points)
    zs = sorted(p[2] for p in points)

    def pct(v, q):
        return v[min(len(v) - 1, max(0, int(q * (len(v) - 1))))]

    print("%d points -> %s" % (len(points), args.out))
    print("full extent    %.2f x %.2f x %.2f m"
          % (xs[-1] - xs[0], ys[-1] - ys[0], zs[-1] - zs[0]))
    print("middle 98%%     %.2f x %.2f x %.2f m"
          % (pct(xs, .99) - pct(xs, .01),
             pct(ys, .99) - pct(ys, .01),
             pct(zs, .99) - pct(zs, .01)))
    print("bounds (98%%)   x %.2f to %.2f   y %.2f to %.2f   z %.2f to %.2f"
          % (pct(xs, .01), pct(xs, .99), pct(ys, .01), pct(ys, .99),
             pct(zs, .01), pct(zs, .99)))
    print("\nOne of the three should be your ceiling height.")


def write_ply(path, points):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write("element vertex %d\n" % len(points))
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for x, y, z in points:
            f.write("%.4f %.4f %.4f\n" % (x, y, z))


def cmd_selftest(_args):
    """Check the geometry, especially the pole invariance."""
    # A point straight along the tilt axis must not move as the axis tilts.
    for beta in (0.0, 37.0, 90.0, 180.0):
        x, y, z = to_3d(0.0, 1000.0, beta)
        assert abs(x - 1.0) < 1e-9, "pole moved in x at beta=%s" % beta
        assert abs(y) < 1e-9 and abs(z) < 1e-9, "pole moved off axis"

    # At beta = 0 the scan plane is the XY plane.
    x, y, z = to_3d(90.0, 1000.0, 0.0)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(z) < 1e-9, "beta=0"

    # At beta = 90 that same bearing must have swung into Z.
    x, y, z = to_3d(90.0, 1000.0, 90.0)
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z - 1.0) < 1e-9, "beta=90"

    # Range must be preserved for every bearing and tilt.
    for theta in (0.0, 17.0, 90.0, 213.0, 359.0):
        for beta in (0.0, 45.0, 123.0):
            x, y, z = to_3d(theta, 2500.0, beta)
            assert abs(math.sqrt(x*x + y*y + z*z) - 2.5) < 1e-9, "range lost"

    # A 180 deg sweep must reach every direction: sample the sphere and
    # confirm each direction is hit by some (theta, beta) pair.
    worst = 0.0
    for target in _fibonacci_sphere(300):
        best = min(_angle_between(target, to_3d(th, 1000.0, be))
                   for be in range(0, 180, 3)
                   for th in range(0, 360, 3))
        worst = max(worst, best)
    assert worst < 4.0, "sweep leaves a gap of %.1f deg" % worst

    print("selftest passed: pole invariance, range preserved,")
    print("full sphere covered by a 180 deg sweep (worst gap %.1f deg)" % worst)


def _fibonacci_sphere(n):
    pts = []
    phi = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(n):
        y = 1.0 - (i / float(n - 1)) * 2.0
        r = math.sqrt(max(0.0, 1.0 - y * y))
        th = phi * i
        pts.append((math.cos(th) * r, y, math.sin(th) * r))
    return pts


def _angle_between(a, b):
    na = math.sqrt(sum(c * c for c in a))
    nb = math.sqrt(sum(c * c for c in b))
    dot = sum(p * q for p, q in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def main():
    p = argparse.ArgumentParser(description="3D room scanner")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan")
    s.add_argument("--lidar", required=True, help="lidar COM port")
    s.add_argument("--arduino", required=True, help="Arduino COM port")
    s.add_argument("--out", default="scan.csv")
    s.add_argument("--steps", type=int, default=STEPS_180)
    s.add_argument("--revs", type=int, default=2,
                   help="lidar revolutions captured per stop")
    s.add_argument("--settle", type=float, default=0.15,
                   help="seconds to wait after each move")
    s.add_argument("--min-range", type=float, default=0.12)
    s.add_argument("--max-range", type=float, default=8.0)
    s.set_defaults(func=cmd_scan)

    fa = sub.add_parser("findaxis")
    fa.add_argument("--raw", required=True)
    fa.add_argument("--min-points", type=int, default=50)
    fa.set_defaults(func=cmd_findaxis)

    ck = sub.add_parser("check")
    ck.add_argument("--raw", required=True)
    ck.add_argument("--offset", type=float, default=0.0)
    ck.add_argument("--sweep", action="store_true",
                    help="try every offset and score planar structure")
    ck.set_defaults(func=cmd_check)

    sl = sub.add_parser("slice")
    sl.add_argument("--raw", required=True)
    sl.add_argument("--offset", type=float, default=0.0)
    sl.add_argument("--beta", type=float, nargs="+",
                    default=[0.0, 45.0, 90.0, 135.0],
                    help="tilt positions to plot, degrees")
    sl.set_defaults(func=cmd_slice)

    v = sub.add_parser("verify")
    v.add_argument("--raw", required=True)
    v.add_argument("--min-overlap", type=int, default=30)
    v.set_defaults(func=cmd_verify)

    m = sub.add_parser("measure")
    m.add_argument("--raw", required=True)
    m.add_argument("--offset", type=float, required=True)
    m.add_argument("--planes", type=int, default=6)
    m.add_argument("--thresh", type=float, default=0.03,
                   help="plane inlier tolerance, metres")
    m.add_argument("--min-inliers", type=int, default=2000)
    m.add_argument("--min-range", type=float, default=0.30,
                   help="ignore close returns, mostly the rig itself")
    m.add_argument("--oy", type=float, default=0.0)
    m.add_argument("--oz", type=float, default=0.0)
    m.add_argument("--gamma", type=float, default=0.0)
    m.set_defaults(func=cmd_measure)

    cal = sub.add_parser("calibrate")
    cal.add_argument("--raw", required=True)
    cal.add_argument("--offset", type=float, required=True)
    cal.add_argument("--planes", type=int, default=6)
    cal.add_argument("--thresh", type=float, default=0.03)
    cal.add_argument("--min-inliers", type=int, default=200)
    cal.add_argument("--min-range", type=float, default=0.30)
    cal.add_argument("--sample", type=int, default=25000)
    cal.add_argument("--span", type=float, default=0.10,
                     help="lever arm search range, metres")
    cal.add_argument("--gamma", type=float, default=4.0,
                     help="misalignment search range, degrees")
    cal.add_argument("--grid", type=int, default=11)
    cal.set_defaults(func=cmd_calibrate)

    b = sub.add_parser("build")
    b.add_argument("--raw", required=True)
    b.add_argument("--out", default="scan.ply")
    b.add_argument("--offset", type=float, default=0.0,
                   help="bearing of the tilt axis, degrees")
    b.add_argument("--oy", type=float, default=0.0,
                   help="lidar offset across the tilt axis, metres")
    b.add_argument("--oz", type=float, default=0.0,
                   help="lidar offset along the scan plane normal, metres")
    b.add_argument("--gamma", type=float, default=0.0,
                   help="scan plane tilt against the axis, degrees")
    b.add_argument("--mirror", action="store_true",
                   help="flip handedness if the room comes out mirrored")
    b.set_defaults(func=cmd_build)

    sub.add_parser("selftest").set_defaults(func=cmd_selftest)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
