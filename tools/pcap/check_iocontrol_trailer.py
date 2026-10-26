#!/usr/bin/env python3
"""Checks the layout of the last 20 bytes of IOControl (step 8 of docs/hardware-testing.md).

Reads a pcapng capture of OpenRobotinoAPI 1 calling Odometry::set() and
NorthStar::setCeilingCal(), prints every distinct trailer with the messages that carry it, and
checks the layout (payload offset 56):

  +0  u8   set odometry
  +1  f32  odometry x
  +5  f32  odometry y
  +9  f32  odometry phi
  +13 f32  NorthStar ceiling calibration (default 1.0)
  +17 3    padding

The expected values default to Odometry::set(1234.5, -678.25, 45) and setCeilingCal(1.25).
Odometry values are also accepted after a unit conversion (mm -> m, deg -> rad); the one that
matched is printed. When a check fails, the expected values are searched at every offset of
the trailers.

Example:
  check_iocontrol_trailer.py capture.pcapng
  check_iocontrol_trailer.py capture.pcapng --odometry 1000 0 90 --ceiling-cal 2.5
"""

import argparse
import math
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcap_extract import reassemble, split_messages  # noqa: E402

IOCONTROL_ID = 0
IOCONTROL_LENGTH = 76
TRAILER_OFFSET = 56
TRAILER_LENGTH = 20
DEFAULT_TRAILER = bytes(13) + struct.pack("<f", 1.0) + bytes(3)

LENGTH_UNITS = {"as given": 1.0, "mm -> m": 1e-3, "m -> mm": 1e3}
ANGLE_UNITS = {"as given": 1.0, "deg -> rad": math.pi / 180, "rad -> deg": 180 / math.pi}


def f32(value):
    """Rounds to float32, as the sender would."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def close(a, b):
    return math.isclose(a, f32(b), rel_tol=1e-6, abs_tol=1e-9)


def decode(trailer):
    flag = trailer[0]
    x, y, phi, cal = struct.unpack_from("<4f", trailer, 1)
    return flag, x, y, phi, cal, trailer[17:]


def describe(trailer):
    flag, x, y, phi, cal, pad = decode(trailer)
    return f"set={flag} x={x:g} y={y:g} phi={phi:g} cal={cal:g} pad={pad.hex(' ')}"


def runs(trailers):
    """Groups consecutive equal trailers into (first index, last index, trailer)."""
    result = []
    for i, t in enumerate(trailers):
        if result and result[-1][2] == t:
            result[-1][1] = i
        else:
            result.append([i, i, t])
    return result


def find_float(trailers, value):
    """Returns the sorted offsets where a float32 close to value appears in any trailer."""
    offsets = set()
    for t in set(trailers):
        for off in range(TRAILER_LENGTH - 3):
            v = struct.unpack_from("<f", t, off)[0]
            if math.isfinite(v) and close(v, value):
                offsets.add(off)
    return sorted(offsets)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", help="pcapng file")
    parser.add_argument("--port", type=int, default=80, help="robot TCP port (default: 80)")
    parser.add_argument("--odometry", nargs=3, type=float, default=[1234.5, -678.25, 45.0],
                        metavar=("X", "Y", "PHI"), help="values passed to Odometry::set()")
    parser.add_argument("--ceiling-cal", type=float, default=1.25,
                        help="value passed to NorthStar::setCeilingCal()")
    args = parser.parse_args()

    trailers = []
    for (src, dst), stream in reassemble(args.capture, args.port).items():
        if dst[1] != args.port:
            continue
        payloads = [raw[3:] for msg_id, raw in split_messages(stream) if msg_id == IOCONTROL_ID]
        if not payloads:
            continue
        print(f"{src[0]}:{src[1]} -> {dst[0]}:{dst[1]}: {len(payloads)} IOControl")
        bad = [p for p in payloads if len(p) != IOCONTROL_LENGTH]
        if bad:
            print(f"  {len(bad)} IOControl with a payload length other than {IOCONTROL_LENGTH}, skipped")
        stream_trailers = [p[TRAILER_OFFSET:] for p in payloads if len(p) == IOCONTROL_LENGTH]
        for first, last, t in runs(stream_trailers):
            print(f"  #{first}-{last} ({last - first + 1}): {t.hex(' ')}")
            print(f"      {describe(t)}")
        trailers += stream_trailers
    if not trailers:
        print("no IOControl in the capture")
        return 1

    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}{': ' + detail if detail else ''}")

    print()
    check("first trailer is the default (cal 1.0, rest zero)", trailers[0] == DEFAULT_TRAILER,
          "" if trailers[0] == DEFAULT_TRAILER else trailers[0].hex(" "))
    nonzero_pad = sorted({t[17:].hex(" ") for t in trailers if any(t[17:])})
    check("padding at +17..+19 is always zero", not nonzero_pad, ", ".join(nonzero_pad))
    flags = sorted({t[0] for t in trailers})
    check("set odometry byte at +0 is 0 or 1", set(flags) <= {0, 1}, f"seen {flags}")

    cal_runs = [r for r in runs(trailers) if close(decode(r[2])[4], args.ceiling_cal)]
    check(f"ceiling calibration {args.ceiling_cal:g} at +13", bool(cal_runs),
          f"in {sum(r[1] - r[0] + 1 for r in cal_runs)} messages" if cal_runs else "")

    x, y, phi = args.odometry
    match = None
    for t in trailers:
        flag, tx, ty, tphi, _, _ = decode(t)
        if flag != 1:
            continue
        for lname, lscale in LENGTH_UNITS.items():
            for aname, ascale in ANGLE_UNITS.items():
                if close(tx, x * lscale) and close(ty, y * lscale) and close(tphi, phi * ascale):
                    match = (lname, aname)
        if match:
            break
    set_count = sum(1 for t in trailers if t[0] == 1)
    check(f"odometry ({x:g}, {y:g}, {phi:g}) at +1/+5/+9 with set byte 1", match is not None,
          f"lengths {match[0]}, angle {match[1]}; set byte 1 in {set_count} messages" if match
          else f"set byte 1 in {set_count} messages")

    if not all(results):
        print("\nwhere the expected values appear (trailer offsets):")
        candidates = [("ceiling cal", args.ceiling_cal, {"as given": 1.0})]
        candidates += [(n, v, LENGTH_UNITS) for n, v in (("x", x), ("y", y))]
        candidates.append(("phi", phi, ANGLE_UNITS))
        found = False
        for name, value, units in candidates:
            for uname, scale in units.items():
                offsets = find_float(trailers, value * scale)
                if offsets and value != 0:
                    found = True
                    print(f"  {name} ({uname}, {f32(value * scale):g}): +{', +'.join(map(str, offsets))}")
        if not found:
            print("  nowhere: were the calls made during this capture?")

    print(f"\n{sum(results)} passed, {len(results) - sum(results)} failed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
