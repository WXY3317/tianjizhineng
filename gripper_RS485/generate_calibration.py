#!/usr/bin/env python3
"""Generate an endpoint-scaled gripper table from the installed vendor model."""
import argparse
import ctypes
import math
from pathlib import Path
import struct

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--model', required=True, type=Path, help='native-architecture libgrip_pawl_lib.so')
parser.add_argument('--open-width-mm', required=True, type=float)
parser.add_argument('--open-angle-rad', required=True, type=float, help='open angle; use 0 if already zeroed')
parser.add_argument('--closed-angle-rad', required=True, type=float, help='closed angle in the same coordinate frame as open angle')
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
width_m = args.open_width_mm / 1000
stroke = args.closed_angle_rad - args.open_angle_rad
if not all(map(math.isfinite, (width_m, stroke))) or not 0.01 <= width_m <= 0.2 or not 1 <= stroke <= 100:
    parser.error('expected width 10..200 mm and positive closing stroke 1..100 rad; verify motor direction and readings')
if args.output.exists():
    parser.error('output exists; choose a new path to avoid replacing an existing calibration')
lib = ctypes.CDLL(str(args.model.resolve()))
memory = ctypes.create_string_buffer(512)
instance = (ctypes.addressof(memory) + 15) & ~15
ctor = getattr(lib, '_ZN9grip_pawlC1Ev')
ctor.argtypes = [ctypes.c_void_p]
ctor.restype = None
fk = getattr(lib, '_ZNK9grip_pawl5fkineEd')
fk.argtypes = [ctypes.c_void_p, ctypes.c_double]
fk.restype = ctypes.c_double
ctor(instance)
radius = struct.unpack('<d', bytes.fromhex('a35a188d5adc343f'))[0]
model_open = fk(instance, 0)
model_closed = fk(instance, stroke * radius)
if not all(map(math.isfinite, (model_open, model_closed))) or model_open <= model_closed:
    parser.error('vendor model is incompatible with measured stroke')
scale = width_m / (model_open - model_closed)
rows = []
for i in range(513):
    angle = stroke * i / 512
    width = scale * (fk(instance, angle * radius) - model_closed)
    rows.append((angle, width))
rows[0] = (0.0, width_m)
rows[-1] = (stroke, 0.0)
if any(not math.isfinite(a) or not math.isfinite(w) for a, w in rows) or any(rows[i+1][1] >= rows[i][1] for i in range(512)):
    parser.error('generated widths are not strictly decreasing; measure intermediate points and fit a different model')
lines = [f'# Open width measured: {args.open_width_mm} mm.',
         f'# Open/closed encoder angles in the same coordinate frame: {args.open_angle_rad}, {args.closed_angle_rad} rad.',
         '# Intermediate widths and forces are model estimates, not measurements.',
         '# Columns: motor_position_rad,width_m,estimated_force_n_per_torque_percent']
for i, (angle, width) in enumerate(rows):
    a, b = rows[max(0, i-1)], rows[min(512, i+1)]
    slope = abs((b[1] - a[1]) / (b[0] - a[0]))
    force_per_percent = 0.9 * 0.23 / (100 * slope)
    lines.append(f'{angle:.17g},{width:.17g},{force_per_percent:.17g}')
args.output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(f'Wrote {args.output}: open={width_m} m, closed_angle={stroke} rad, 513 points')
