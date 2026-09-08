"""
Minimal YDLIDAR X2L driver: raw serial -> (angle_deg, distance_mm) points.

Protocol per the YDLIDAR X2 Development Manual v1.2.
The lidar starts ranging automatically on power-up; no start command is sent.

Usage:
    python x2l.py --port COM4 plot
    python x2l.py --port /dev/ttyUSB0 ruler --bearing 0 --n 30
    python x2l.py selftest
"""

import argparse
import math
import statistics
import struct
import sys

# The packet header is the 16-bit value 0x55AA sent low byte first,
# so on the wire it is AA then 55.
HEADER = b"\xaa\x55"

# Constants from the second-level angle correction formula in the manual.
_K1 = 21.8
_K2 = 155.3


def angle_correction_deg(distance_mm):
    """Second-level angle correction. Zero for invalid (zero) returns."""
    if distance_mm <= 0:
        return 0.0
    return math.degrees(
        math.atan(_K1 * (_K2 - distance_mm) / (_K2 * distance_mm))
    )


def parse_packet(pkt, correct=True):
    """
    Parse one scan packet.

    Returns (is_lap_start, [(angle_deg, distance_mm), ...]) or None if the
    checksum fails.

    Layout:  PH(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2) S1(2) ... SN(2)
    """
    ph, ct_lsn, fsa, lsa, cs = struct.unpack("<HHHHH", pkt[:10])
    lsn = (ct_lsn >> 8) & 0xFF
    ct = ct_lsn & 0xFF

    samples = struct.unpack("<%dH" % lsn, pkt[10:10 + 2 * lsn])

    # Checksum is the XOR of every 16-bit word in the packet except CS itself.
    chk = ph ^ ct_lsn ^ fsa ^ lsa
    for s in samples:
        chk ^= s
    if chk != cs:
        return None

    # First-level analysis: bit 0 of each angle field is a parity bit.
    a_start = (fsa >> 1) / 64.0
    a_end = (lsa >> 1) / 64.0
    span = a_end - a_start
    if span < 0:
        span += 360.0

    points = []
    for i, s in enumerate(samples):
        distance = s / 4.0
        angle = a_start + (span * i / (lsn - 1) if lsn > 1 else 0.0)
        if correct:
            angle += angle_correction_deg(distance)
        points.append((angle % 360.0, distance))

    # CT bit 0 marks the first packet of a new revolution.
    return bool(ct & 1), points


class X2L:
    def __init__(self, port, baud=115200, timeout=1.0, correct=True):
        import serial
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buf = bytearray()
        self.correct = correct
        self.stats = {"ok": 0, "bad_checksum": 0}

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def packets(self):
        """Yield (is_lap_start, points) for every valid packet."""
        while True:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                self.buf.extend(chunk)

            while True:
                i = self.buf.find(HEADER)
                if i < 0:
                    # Keep one byte in case a header straddles the boundary.
                    del self.buf[:-1]
                    break
                if len(self.buf) < i + 10:
                    del self.buf[:i]
                    break
                lsn = self.buf[i + 3]
                length = 10 + 2 * lsn
                if len(self.buf) < i + length:
                    del self.buf[:i]
                    break

                pkt = bytes(self.buf[i:i + length])
                del self.buf[:i + length]

                result = parse_packet(pkt, self.correct)
                if result is None:
                    self.stats["bad_checksum"] += 1
                else:
                    self.stats["ok"] += 1
                    yield result

    def scans(self, skip_first=True):
        """Yield one complete revolution at a time as a list of points."""
        current = []
        started = not skip_first
        for is_start, points in self.packets():
            if is_start:
                if current and started:
                    yield current
                started = True
                current = []
            if started:
                current.extend(points)


def shift(angle_deg, offset_deg):
    """Re-reference a raw bearing so the physical front of the unit reads 0."""
    return (angle_deg - offset_deg) % 360.0


def to_xy(angle_deg, distance_mm):
    """Bearing measured clockwise from straight ahead -> (x right, y forward), metres."""
    r = math.radians(angle_deg)
    d = distance_mm / 1000.0
    return d * math.sin(r), d * math.cos(r)


def cmd_xy(args):
    """Cartesian view. Flat walls should look flat here; in polar they never do."""
    import matplotlib.pyplot as plt
    import numpy as np

    limit = args.range
    with X2L(args.port, correct=not args.no_correction) as lidar:
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.set_aspect("equal")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_xlabel("x, metres")
        ax.set_ylabel("y, metres (0 deg is straight ahead)")
        ax.grid(True, alpha=0.3)
        ax.axhline(0, lw=0.5, color="0.7")
        ax.axvline(0, lw=0.5, color="0.7")
        scatter = ax.scatter([], [], s=3)
        plt.ion()
        plt.show()

        for scan in lidar.scans():
            pts = [
                to_xy(shift(a, args.offset), d)
                for a, d in scan
                if 120.0 < d <= 8000.0
            ]
            if not pts:
                continue
            scatter.set_offsets(np.asarray(pts))
            ax.set_title("%d points   bad checksums: %d"
                         % (len(pts), lidar.stats["bad_checksum"]))
            plt.pause(0.001)
            if not plt.fignum_exists(fig.number):
                break


def _fit_window(P):
    """Total-least-squares line fit. Returns (rms_mm, extent_m)."""
    import numpy as np
    centroid = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - centroid, full_matrices=False)
    resid = (P - centroid) @ vt[1] * 1000.0
    extent = float(np.ptp((P - centroid) @ vt[0]))
    return float(np.sqrt((resid ** 2).mean())), extent


def cmd_survey(args):
    """
    Look all the way round, then report which bearings are actually usable and
    which angular window contains the flattest surface. Use this to pick a
    --bearing for the wall test instead of guessing.
    """
    import numpy as np

    raw, valid = 0, []
    with X2L(args.port, correct=not args.no_correction) as lidar:
        for n, scan in enumerate(lidar.scans(), 1):
            raw += len(scan)
            valid.extend((shift(a, args.offset), d)
                         for a, d in scan if 120.0 < d <= 8000.0)
            if n >= args.n:
                break

    if not valid:
        print("No valid returns at all.")
        return

    print("%d of %d rays returned (%.0f%%) over %d revolutions\n"
          % (len(valid), raw, 100.0 * len(valid) / raw, args.n))

    bearings = np.array([b for b, _ in valid])
    ranges = np.array([d for _, d in valid]) / 1000.0
    expected = raw / 24.0

    print("sector      returns   median range")
    for k in range(24):
        lo, hi = k * 15.0, (k + 1) * 15.0
        sel = (bearings >= lo) & (bearings < hi)
        hits = int(sel.sum())
        bar = "#" * int(round(10.0 * min(hits / expected, 1.0)))
        med = ("%6.2f m" % np.median(ranges[sel])) if hits else "     --"
        print("%3.0f-%3.0f deg  %4d %-11s %s" % (lo, hi, hits, bar, med))

    xy = np.array([to_xy(b, d) for b, d in valid])
    results = []
    for centre in range(0, 360, 2):
        delta = (bearings - centre + 180.0) % 360.0 - 180.0
        sel = np.abs(delta) <= args.window / 2.0
        if sel.sum() < 40:
            continue
        rms, extent = _fit_window(xy[sel])
        if extent >= 0.4:
            results.append((rms, centre, int(sel.sum()), extent))

    if not results:
        print("\nNo window held a surface big enough to fit.")
        return

    results.sort()
    print("\nFlattest %.0f deg windows:" % args.window)
    print("bearing   rms      extent   points")
    for rms, centre, npts, extent in results[:6]:
        print("%5d   %6.1f mm  %5.2f m  %5d" % (centre, rms, extent, npts))
    print("\nTry:  wall --bearing %d --window %.0f --plot"
          % (results[0][1], args.window))


def cmd_wall(args):
    """
    Point a chosen bearing window at a flat surface and measure how flat the
    data says it is. Fits a line by total least squares and reports residuals.

    A large, systematically bowed residual means the angle correction is being
    applied wrongly. Scattered residuals are just sensor noise.
    """
    import numpy as np

    half = args.window / 2.0
    pts = []
    with X2L(args.port, correct=not args.no_correction) as lidar:
        for n, scan in enumerate(lidar.scans(), 1):
            for a, d in scan:
                b = shift(a, args.offset)
                if not (args.min_range * 1000.0 < d <= args.max_range * 1000.0):
                    continue
                if abs((b - args.bearing + 180.0) % 360.0 - 180.0) <= half:
                    pts.append(to_xy(b, d))
            if n >= args.n:
                break

    if len(pts) < 10:
        print("Only %d points in that window. Widen --window or check --bearing."
              % len(pts))
        return

    P = np.asarray(pts)
    rms, extent_m = _fit_window(P)
    centroid = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - centroid, full_matrices=False)
    along, normal = vt[0], vt[1]
    resid_mm = (P - centroid) @ normal * 1000.0

    mean_range = float(np.linalg.norm(centroid))
    expected = 2.0 * mean_range * math.tan(math.radians(half))

    print("points           %d over %d revolutions" % (len(pts), args.n))
    print("surface extent   %.2f m  (expected ~%.2f m for this window)"
          % (extent_m, expected))
    print("mean range       %.2f m" % mean_range)
    print("angle correction %s" % ("off" if args.no_correction else "on"))
    print("rms residual     %.1f mm" % rms)
    print("max deviation    %.1f mm" % float(np.abs(resid_mm).max()))

    if extent_m > 1.5 * expected:
        print("\nWARNING: the fitted extent is far larger than this window can")
        print("span at this range, so it is holding more than one surface.")
        print("Narrow --window, or gate the background out with --max-range.")
        print("The rms figure above is meaningless as a flatness measure.")

    if args.plot:
        import matplotlib.pyplot as plt
        s = (P - centroid) @ along
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.scatter(s, resid_mm, s=6)
        ax.axhline(0, lw=0.8, color="0.5")
        ax.set_xlabel("position along the surface, m")
        ax.set_ylabel("deviation from best-fit line, mm")
        ax.set_title("rms %.1f mm, correction %s"
                     % (rms, "off" if args.no_correction else "on"))
        ax.grid(True, alpha=0.3)
        plt.show()


def cmd_plot(args):
    import matplotlib.pyplot as plt
    import numpy as np

    with X2L(args.port, correct=not args.no_correction) as lidar:
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="polar")
        ax.set_theta_direction(-1)          # YDLIDAR angles increase clockwise
        ax.set_theta_zero_location("N")
        ax.set_rmax(8.0)
        ax.grid(True, alpha=0.3)
        scatter = ax.scatter([], [], s=3)
        plt.ion()
        plt.show()

        for scan in lidar.scans():
            pts = [
                (math.radians(a), d / 1000.0)
                for a, d in scan
                if 120.0 < d <= 8000.0
            ]
            if not pts:
                continue
            scatter.set_offsets(np.asarray(pts))
            bad = lidar.stats["bad_checksum"]
            ax.set_title("%d points   bad checksums: %d" % (len(pts), bad))
            plt.pause(0.001)
            if not plt.fignum_exists(fig.number):
                break


def cmd_ruler(args):
    """
    Point the lidar at a flat surface and measure it. Compare against a tape
    measure at several distances to characterise the sensor's error.
    """
    half = args.window / 2.0
    readings = []

    with X2L(args.port) as lidar:
        for n, scan in enumerate(lidar.scans()):
            hits = [
                d for a, d in scan
                if d > 0 and abs((shift(a, args.offset) - args.bearing + 180.0)
                                 % 360.0 - 180.0) <= half
            ]
            if hits:
                readings.append(statistics.median(hits))
            if len(readings) >= args.n:
                break

    if not readings:
        print("No returns in that angular window. Check the bearing.")
        return

    mean = statistics.mean(readings)
    sd = statistics.pstdev(readings) if len(readings) > 1 else 0.0
    print("bearing      %.1f deg (+/- %.1f)" % (args.bearing, half))
    print("revolutions  %d" % len(readings))
    print("mean         %.1f mm" % mean)
    print("std dev      %.1f mm  (%.2f%% of mean)" % (sd, 100.0 * sd / mean))
    print("min / max    %.1f / %.1f mm" % (min(readings), max(readings)))


def cmd_selftest(_args):
    """Verify the parser against the worked example in the X2 manual."""
    # Manual: bytes 4..8 are 28 E5 6F BD 79, giving
    # LSN = 0x28 = 40, FSA = 0x6FE5, LSA = 0x79BD,
    # Angle_FSA = 223.78 deg, Angle_LSA = 243.47 deg,
    # sample 0x6FE5 -> 7161.25 mm,
    # AngCorrect(1000mm) = -6.7622 deg, AngCorrect(8000mm) = -7.8374 deg.
    fsa, lsa = 0x6FE5, 0x79BD
    assert abs((fsa >> 1) / 64.0 - 223.78) < 0.01, "start angle"
    assert abs((lsa >> 1) / 64.0 - 243.47) < 0.01, "end angle"
    assert abs(0x6FE5 / 4.0 - 7161.25) < 0.01, "distance"
    assert abs(angle_correction_deg(1000) - (-6.7622)) < 0.001, "corr @1000mm"
    assert abs(angle_correction_deg(8000) - (-7.8374)) < 0.002, "corr @8000mm"
    assert angle_correction_deg(0) == 0.0, "corr @invalid"

    # Round-trip a synthetic packet through the real parser.
    lsn = 3
    samples = (4000, 4400, 4800)          # 1000.0, 1100.0, 1200.0 mm
    ph, ct_lsn = 0x55AA, (lsn << 8) | 0x00
    chk = ph ^ ct_lsn ^ fsa ^ lsa
    for s in samples:
        chk ^= s
    pkt = struct.pack("<HHHHH", ph, ct_lsn, fsa, lsa, chk)
    pkt += struct.pack("<3H", *samples)

    out = parse_packet(pkt)
    assert out is not None, "checksum rejected a valid packet"
    is_start, points = out
    assert is_start is False, "CT bit 0"
    assert len(points) == lsn, "sample count"
    assert abs(points[0][1] - 1000.0) < 1e-6, "decoded distance"
    assert abs(points[0][0] - (223.78 - 6.7622)) < 0.02, "corrected angle"

    # And confirm a corrupted packet is rejected.
    bad = bytearray(pkt)
    bad[11] ^= 0xFF
    assert parse_packet(bytes(bad)) is None, "corruption not detected"

    print("selftest passed: angles, distances, correction, checksum")


def main():
    p = argparse.ArgumentParser(description="YDLIDAR X2L reader")
    p.add_argument("--port", help="e.g. COM4 or /dev/ttyUSB0")
    p.add_argument("--offset", type=float, default=0.0,
                   help="raw bearing of the physical front, degrees")
    p.add_argument("--no-correction", action="store_true",
                   help="skip the second-level angle correction")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("plot").set_defaults(func=cmd_plot)

    x = sub.add_parser("xy")
    x.add_argument("--range", type=float, default=5.0, help="axis limit, metres")
    x.set_defaults(func=cmd_xy)

    s = sub.add_parser("survey")
    s.add_argument("--window", type=float, default=40.0,
                   help="candidate window width, degrees")
    s.add_argument("--n", type=int, default=20, help="revolutions to collect")
    s.set_defaults(func=cmd_survey)

    w = sub.add_parser("wall")
    w.add_argument("--bearing", type=float, default=0.0)
    w.add_argument("--window", type=float, default=40.0,
                   help="angular width of surface to fit, degrees")
    w.add_argument("--n", type=int, default=20, help="revolutions to collect")
    w.add_argument("--min-range", type=float, default=0.12, help="metres")
    w.add_argument("--max-range", type=float, default=8.0,
                   help="metres; use this to gate out the background")
    w.add_argument("--plot", action="store_true", help="show residuals")
    w.set_defaults(func=cmd_wall)

    r = sub.add_parser("ruler")
    r.add_argument("--bearing", type=float, default=0.0,
                   help="direction to measure, degrees")
    r.add_argument("--window", type=float, default=2.0,
                   help="angular width to average over, degrees")
    r.add_argument("--n", type=int, default=30, help="revolutions to collect")
    r.set_defaults(func=cmd_ruler)

    sub.add_parser("selftest").set_defaults(func=cmd_selftest)

    args = p.parse_args()
    if args.cmd in ("plot", "xy", "wall", "ruler", "survey") and not args.port:
        p.error("--port is required for %s" % args.cmd)
    args.func(args)


if __name__ == "__main__":
    main()
