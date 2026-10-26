"""Offline corner audit/numbered SVG overlay. No FPGA resources or serial writes.

Images, if supplied, MUST be the exact saved calibration frames at native size,
not a later live frame or a photograph of the HDMI monitor. Pillow is optional
and required only for validating supplied image dimensions.
"""
import argparse
import base64
import html
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def image_size(data, width=None, height=None):
    geometry = data.get('geometry', {})
    size = geometry['processing_size'] if geometry.get('supported') else (1280, 720)
    return width if width is not None else size[0], height if height is not None else size[1]


def spatial_metrics(points, width, height, residuals=None):
    """Disjoint image regions; empty regions stay unmeasured, never zero-error.

    Optional residuals are interleaved dx/dy from a fit on THESE observations.
    They measure training reprojection error, not independent correction error.
    """
    if residuals is not None and len(residuals) != 2*len(points):
        raise ValueError('Residual count does not match points')
    regions = {name: [] for name in ('center', 'outer_band', 'outer_corners')}
    cells = [[0]*4 for _ in range(3)]
    for i, (x, y) in enumerate(points):
        if not (math.isfinite(x) and math.isfinite(y) and 0 <= x < width and 0 <= y < height):
            continue
        ex = x <= (width-1)/4 or x >= 3*(width-1)/4
        ey = y <= (height-1)/4 or y >= 3*(height-1)/4
        name = 'outer_corners' if ex and ey else 'outer_band' if ex or ey else 'center'
        regions[name].append(i)
        cells[min(2, int(y*3/height))][min(3, int(x*4/width))] += 1
    stats = {}
    for name, indices in regions.items():
        errors = ([math.hypot(residuals[2*i], residuals[2*i+1]) for i in indices]
                  if residuals is not None else [])
        stats[name] = dict(count=len(indices),
                           training_rms_px=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
                           training_max_px=max(errors) if errors else None)
    return dict(regions=stats, grid_counts_top_to_bottom=cells,
                empty_grid_cells=sum(n == 0 for row in cells for n in row),
                error_scope='training reprojection only; empty regions have no accuracy evidence')


def coverage(points, width, height):
    """Match build_init.py: >=4 corners in EACH outer quarter-band.

    This conservative image-space heuristic is NOT a test of parameter
    identifiability, correct corner localization, or sufficient pose diversity.
    """
    counts = dict(left=0, right=0, top=0, bottom=0)
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y) and
                0 <= x < width and 0 <= y < height):
            continue
        counts['left'] += x <= (width-1)/4
        counts['right'] += x >= 3*(width-1)/4
        counts['top'] += y <= (height-1)/4
        counts['bottom'] += y >= 3*(height-1)/4
    return dict(edge_counts=counts, minimum_per_edge=4,
                coverage_ok=all(n >= 4 for n in counts.values()))


def audit(data, width=None, height=None, rows=5, cols=8):
    width, height = image_size(data, width, height)
    if width < 2 or height < 2 or rows < 2 or cols < 2:
        raise ValueError('Image and grid dimensions must be at least 2')
    entries = data['corners']
    n = rows*cols
    if not entries or len(entries) % n or len(entries)//n < 3:
        raise ValueError('Need complete views (at least 3); check rows/cols and corner_count')
    if [e['index'] for e in entries] != list(range(len(entries))):
        raise ValueError('Corner indices must be contiguous from zero')
    points = [tuple(map(float, e['xy'])) for e in entries]
    if any(len(p) != 2 for p in points):
        raise ValueError('Each corner needs exactly two coordinates')
    views, errors = [], []
    for v in range(len(points)//n):
        p = points[v*n:(v+1)*n]
        bad = [i for i, (x, y) in enumerate(p) if not
               (math.isfinite(x) and math.isfinite(y) and 0 <= x < width and 0 <= y < height)]
        if bad:
            errors.append(dict(view=v, reason='nonfinite_or_out_of_image', points=bad))
            views.append(dict(view=v, invalid_points=bad))
            continue
        signs, minimum = set(), float('inf')
        for r in range(rows):
            for c in range(cols):
                a = p[r*cols+c]
                for other in ((r*cols+c+1,) if c+1 < cols else ()) + ((r*cols+c+cols,) if r+1 < rows else ()):
                    b = p[other]
                    minimum = min(minimum, math.hypot(a[0]-b[0], a[1]-b[1]))
                if r+1 < rows and c+1 < cols:
                    b, d, e = p[r*cols+c+1], p[(r+1)*cols+c], p[(r+1)*cols+c+1]
                    for u, w in (((b[0]-a[0], b[1]-a[1]), (d[0]-a[0], d[1]-a[1])),
                                 ((d[0]-e[0], d[1]-e[1]), (b[0]-e[0], b[1]-e[1]))):
                        cross = u[0]*w[1]-u[1]*w[0]
                        signs.add(0 if abs(cross) < 1e-3 else (1 if cross > 0 else -1))
        if len(signs) != 1 or 0 in signs or minimum < 1:
            errors.append(dict(view=v, reason='grid_order_or_degenerate_spacing',
                               cell_winding=sorted(signs), minimum_neighbor_pixels=minimum))
        xs, ys = zip(*p)
        views.append(dict(view=v, bbox=[min(xs), min(ys), max(xs), max(ys)],
                          span_fraction=[(max(xs)-min(xs))/width, (max(ys)-min(ys))/height],
                          minimum_neighbor_pixels=minimum, **coverage(p, width, height)))
    policy = coverage(points, width, height)
    return dict(width=width, height=height, rows=rows, cols=cols, views=views,
                errors=errors, corner_geometry_ok=not errors, **policy,
                selected_model='regularized_full',
                spatial=spatial_metrics(points, width, height),
                notes=['Coverage is a heuristic, not proof of identifiability or accuracy.',
                       'Inspect numbered overlays, vary board tilt, and validate on independent image regions.',
                       'Do not replace fitted parameters with artificial-distortion presets.'])


def overlay(points, width, height, rows, cols, image=None):
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
             f'<rect width="{width}" height="{height}" fill="#202830"/>']
    if image:
        from PIL import Image
        with Image.open(image) as frame:
            if frame.size != (width, height):
                raise ValueError(f'{image}: expected native {width}x{height}, got {frame.size}; will not resize')
            mime = Image.MIME[frame.format]
        payload = base64.b64encode(image.read_bytes()).decode('ascii')
        parts.append(f'<image width="{width}" height="{height}" href="data:{mime};base64,{payload}"/>')
    for x in ((width-1)/4, 3*(width-1)/4):
        parts.append(f'<path d="M{x} 0V{height}" stroke="#fbc531" stroke-dasharray="8 8"/>')
    for y in ((height-1)/4, 3*(height-1)/4):
        parts.append(f'<path d="M0 {y}H{width}" stroke="#fbc531" stroke-dasharray="8 8"/>')
    colors = ['#00ffb3', '#ffe066', '#42c9ff', '#ff82cc', '#b8ff59']
    for r in range(rows):
        color = colors[r % len(colors)]
        line = ' '.join(f'{x},{y}' for x, y in points[r*cols:(r+1)*cols] if math.isfinite(x) and math.isfinite(y))
        parts.append(f'<polyline points="{html.escape(line)}" fill="none" stroke="{color}" stroke-width="1.5"/>')
    for i, (x, y) in enumerate(points):
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        color = colors[(i//cols) % len(colors)]
        parts += [f'<circle cx="{x}" cy="{y}" r="4" fill="none" stroke="{color}" stroke-width="2"/>',
                  f'<text x="{x+5}" y="{y-5}" fill="{color}" stroke="#000" stroke-width=".5" paint-order="stroke" font-size="14">{i}</text>']
    parts.append('</svg>')
    return '\n'.join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('corners', type=Path)
    parser.add_argument('--width', type=int)
    parser.add_argument('--height', type=int)
    parser.add_argument('--rows', type=int, default=5)
    parser.add_argument('--cols', type=int, default=8)
    parser.add_argument('--images', nargs='+', type=Path)
    parser.add_argument('--out', type=Path, default=ROOT/'build/debug/corner_audit')
    args = parser.parse_args()
    data = json.loads(args.corners.read_text(encoding='utf-8'))
    args.width, args.height = image_size(data, args.width, args.height)
    report = audit(data, args.width, args.height, args.rows, args.cols)
    n = args.rows*args.cols
    count = len(report['views'])
    if args.images and len(args.images) != count:
        parser.error(f'Need exactly {count} original images in view order')
    args.out.mkdir(parents=True, exist_ok=True)
    for v in range(count):
        points = [e['xy'] for e in data['corners'][v*n:(v+1)*n]]
        svg = overlay(points, args.width, args.height, args.rows, args.cols,
                      args.images[v] if args.images else None)
        (args.out/f'view{v}.svg').write_text(svg, encoding='utf-8')
    (args.out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(f'Numbered overlays: {args.out}')
    if report['errors']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
