#!/usr/bin/env python3
"""Correct a gripper CSV using measured actual widths at commanded targets."""
import argparse
import csv
import math
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', required=True, type=Path)
parser.add_argument('--output', required=True, type=Path)
parser.add_argument('--point', action='append', required=True, help='target_mm:measured_mm; repeat')
args = parser.parse_args()
if args.output.exists():
    parser.error('output exists; choose a new path')
comments, rows = [], []
for line in args.input.read_text(encoding='utf-8').splitlines():
    if line.startswith('#'):
        comments.append(line)
    elif line.strip():
        values = next(csv.reader([line]))
        if len(values) != 3:
            parser.error('input must contain angle,width,force columns')
        rows.append(tuple(map(float, values)))
if len(rows) < 3 or not all(math.isfinite(v) for row in rows for v in row):
    parser.error('invalid input table')
maximum_mm = rows[0][1] * 1000
anchors = [(0.0, 0.0)]
for item in args.point:
    try:
        target, measured = map(float, item.split(':'))
    except ValueError:
        parser.error('point must be target_mm:measured_mm')
    if not all(map(math.isfinite, (target, measured))) or not 0 < target < maximum_mm or not 0 < measured < maximum_mm:
        parser.error('point outside open/closed endpoints')
    anchors.append((target, measured))
anchors.append((maximum_mm, maximum_mm))
anchors.sort()
if any(anchors[i][0] >= anchors[i+1][0] or anchors[i][1] >= anchors[i+1][1]
       for i in range(len(anchors)-1)):
    parser.error('anchor measurements must be strictly increasing in target and actual width')

def actual_width(model_width_m):
    model_mm = model_width_m * 1000
    for i in range(len(anchors)-1):
        x0, y0 = anchors[i]
        x1, y1 = anchors[i+1]
        if model_mm <= x1 or i == len(anchors)-2:
            return (y0 + (model_mm-x0) * (y1-y0) / (x1-x0)) / 1000
    raise AssertionError

corrected = [(angle, actual_width(width), force) for angle, width, force in rows]
if any(corrected[i][0] >= corrected[i+1][0] or corrected[i][1] <= corrected[i+1][1]
       for i in range(len(corrected)-1)):
    parser.error('corrected table is not strictly monotonic')
# Force is derived from local width/angle slope; recompute after changing width.
lines = comments + ['# Corrected using target_mm:measured_mm anchors: ' + ', '.join(f'{x:g}:{y:g}' for x,y in anchors)]
for i, (angle, width, _) in enumerate(corrected):
    a, b = corrected[max(0,i-1)], corrected[min(len(corrected)-1,i+1)]
    slope = abs((b[1]-a[1])/(b[0]-a[0]))
    coefficient = 0.9 * 0.23 / (100 * slope)
    lines.append(f'{angle:.17g},{width:.17g},{coefficient:.17g}')
args.output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(f'Wrote {args.output} with {len(rows)} points; anchors={anchors}')
