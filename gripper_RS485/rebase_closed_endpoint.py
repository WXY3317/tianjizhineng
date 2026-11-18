#!/usr/bin/env python3
"""Rebase the unmeasured tail of a gripper calibration to a measured closed angle."""
import argparse
import bisect
import csv
import math
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', required=True, type=Path)
parser.add_argument('--output', required=True, type=Path)
parser.add_argument('--anchor-angle-rad', required=True, type=float,
                    help='last measured interior angle retained without remapping')
parser.add_argument('--closed-angle-rad', required=True, type=float)
args = parser.parse_args()
if args.output.exists():
    parser.error('output exists; choose a new path')

comments, old = [], []
for line in args.input.read_text(encoding='utf-8').splitlines():
    if line.startswith('#'):
        if not line.startswith(('# Open/closed encoder angles', '# Rebooted-device fit:')):
            comments.append(line)
    elif line.strip():
        values = next(csv.reader([line]))
        if len(values) != 3:
            parser.error('input must contain angle,width,force columns')
        old.append(tuple(map(float, values)))
if len(old) < 3 or not all(math.isfinite(v) for row in old for v in row):
    parser.error('invalid input table')
old_closed = old[-1][0]
anchor = args.anchor_angle_rad
new_closed = args.closed_angle_rad
if not all(map(math.isfinite, (anchor, new_closed))) or not 0 < anchor < new_closed < old_closed:
    parser.error('expected 0 < anchor angle < measured closed angle < old closed angle')
angles = [row[0] for row in old]
if angles[0] != 0 or any(a >= b for a, b in zip(angles, angles[1:])):
    parser.error('input angles must start at zero and increase')

def old_width(angle):
    i = max(0, min(len(old)-2, bisect.bisect_right(angles, angle)-1))
    a0, w0, _ = old[i]
    a1, w1, _ = old[i+1]
    return w0 + (angle-a0)*(w1-w0)/(a1-a0)

rows = []
for i in range(len(old)):
    new_angle = new_closed * i / (len(old)-1)
    source_angle = (new_angle if new_angle <= anchor else
                    anchor + (new_angle-anchor)*(old_closed-anchor)/(new_closed-anchor))
    rows.append((new_angle, old_width(source_angle)))
rows[0] = (0.0, old[0][1])
rows[-1] = (new_closed, 0.0)
if any(rows[i+1][1] >= rows[i][1] for i in range(len(rows)-1)):
    parser.error('rebased widths are not strictly decreasing')

comments.append(f'# Closed endpoint confirmed by physical contact: {new_closed:.12g} rad.')
comments.append(f'# Angles up to {anchor:.12g} rad retain the measured interior mapping; only the unmeasured closed tail was rebased.')
for i, (angle, width) in enumerate(rows):
    a, b = rows[max(0, i-1)], rows[min(len(rows)-1, i+1)]
    slope = abs((b[1]-a[1])/(b[0]-a[0]))
    force_per_percent = 0.9 * 0.23 / (100 * slope)
    comments.append(f'{angle:.17g},{width:.17g},{force_per_percent:.17g}')
args.output.write_text('\n'.join(comments) + '\n', encoding='utf-8')
print(f'Wrote {args.output}: closed angle {new_closed:.12g} rad, {len(rows)} points')
