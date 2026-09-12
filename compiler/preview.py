"""Text preview of a voxel structure.

The preview exists so a proposal can be inspected before anything is built. It
renders the model as given; it does not check or repair it.
"""
from schema import validate

GLYPHS = {'red': 'R', 'green': 'G', 'blue': 'B', 'yellow': 'Y', 'orange': 'O', 'white': 'W'}


def layer_view(structure, z):
    """One layer seen from above, with +x to the right and +y upward."""
    cells = {(v.x, v.y): v.color for v in structure.voxels if v.z == z}
    if not cells:
        return '(empty layer)'
    xs = [x for x, _ in cells]
    ys = [y for _, y in cells]
    rows = []
    for y in range(max(ys), min(ys)-1, -1):
        rows.append(''.join(GLYPHS.get(cells.get((x, y)), '.') if (x, y) in cells else '.'
                            for x in range(min(xs), max(xs)+1)))
    return '\n'.join(rows)


def render(structure, report=None):
    """Full text preview: header, per-layer maps, counts, and any problems."""
    report = report or validate(structure)
    lines = [f'{structure.name or "structure"}  [{structure.structure_id}]',
             f'source: {structure.source}']
    if structure.prompt:
        lines.append(f'prompt: {structure.prompt!r}')
    layers = sorted({v.z for v in structure.voxels})
    for z in layers:
        lines += ['', f'layer z={z} (top view)', layer_view(structure, z)]
    counts = report.counts
    lines += ['', f'cubes: {counts.get("cubes", 0)}  layers: {counts.get("layers", 0)}  '
                  f'footprint: {counts.get("footprint", "-")}']
    if counts.get('colors'):
        lines.append('colors: '+', '.join(f'{n} {c}' for c, n in counts['colors'].items()))
    if report.ok:
        lines.append('checks: passed (schema, budget, support, connectivity)')
    else:
        lines.append(f'checks: REJECTED ({len(report.problems)} problems)')
        lines += ['  '+str(p) for p in report.problems[:20]]
        if len(report.problems) > 20:
            lines.append(f'  ... {len(report.problems)-20} more')
    return '\n'.join(lines)
