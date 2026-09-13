"""Reduce a picture-shaped grid to something the arm can actually place.

Two rules decide what is buildable, and a picture obeys neither by default:

- direct support only, so nothing may float or overhang (`schema.py`);
- the gripper closes two opposed pads on a cube, so each pad needs the whole
  neighbouring cell. A region two or more cells wide in **both** x and y can
  never be finished, because the cube placed last in it is boxed in
  (`placement.py`).

A downsampled photograph is a solid silhouette, so it fails the second rule
everywhere. What survives is line art: a one-cube-wide outline. This module
performs that reduction deterministically, and returns a record of every cube it
dropped. A simplified structure is a simplification, not the picture; callers are
expected to show what changed rather than imply the robot built the photograph.

The reduction is a heuristic, not an optimiser. It does not claim to find the
most recognisable buildable subset, and it never decides acceptability itself:
`schema.validate` still has the last word.
"""
from placement import sequence

STEPS_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def _neighbours(cell):
    x, y, z = cell
    for dx, dy in STEPS_4:
        yield (x+dx, y+dy, z)
    yield (x, y, z+1)
    yield (x, y, z-1)


def outline(cells):
    """Cells on the boundary of the silhouette: any of the four in-layer
    neighbours empty. Kept as the plain definition of boundary; `contour` is what
    the reduction uses, because a boundary set can fall apart when thinned."""
    return {cell: colour for cell, colour in cells.items()
            if any((cell[0]+dx, cell[1]+dy, cell[2]) not in cells for dx, dy in STEPS_4)}


# Vertex (px, py) is the lower-left corner of cell (px, py). Walking an edge in
# direction d, these are the cells on the left and right of the walker.
_SIDES = {(1, 0): ((0, 0), (0, -1)), (0, 1): ((-1, 0), (0, 0)),
          (-1, 0): ((-1, -1), (-1, 0)), (0, -1): ((0, -1), (-1, -1))}
_LEFT = {(1, 0): (0, 1), (0, 1): (-1, 0), (-1, 0): (0, -1), (0, -1): (1, 0)}
_RIGHT = {d: l for l, d in _LEFT.items()}


def contour(cells):
    """Walk the outer boundary of each layer and keep the cells the walk touches.

    This follows the crack between filled and empty cells with the shape on the
    left. Consecutive steps stay on the same cell or move to a face neighbour, so
    the result is a closed, 4-connected loop **by construction** — which
    `outline` is not: thinning a boundary set can cut it in two, and then the
    "largest connected piece" rule throws most of the subject away.

    Interior holes are ignored: what comes out is the silhouette's outer edge,
    which is the line art a two-pad gripper can actually place.
    """
    kept = {}
    for z in sorted({cell[2] for cell in cells}):
        layer = {(cell[0], cell[1]) for cell in cells if cell[2] == z}
        remaining = set(layer)
        while remaining:
            start_cell = min(remaining, key=lambda c: (c[1], c[0]))
            walked = _walk(layer, start_cell)
            # Everything enclosed by this walk is accounted for; anything left
            # belongs to a separate blob, which `connected` will judge later.
            remaining -= set(walked) | _enclosed(layer, set(walked))
            for cell in _close_diagonals(walked, layer):
                # A cell inserted at a pinch point is not in the silhouette; it is
                # reported as an addition by `make_buildable`.
                source = (cell[0], cell[1], z)
                kept[source] = cells.get(source) or _borrow_colour(cells, source)
    return kept


def _close_diagonals(walked, layer):
    """Insert a cell wherever the walk steps diagonally.

    Two filled cells touching only at a corner, with empty cells on the other
    diagonal, is the one place the crack walk changes cell without crossing a
    face. Left alone it becomes a `disconnected` rejection that throws away half
    the outline, so the corner is filled in: the shared orthogonal neighbour that
    is part of the silhouette if there is one, otherwise the lower-left of the two.
    """
    out = list(walked)
    for a, b in zip(walked, walked[1:] + walked[:1]):
        if abs(a[0]-b[0]) == 1 and abs(a[1]-b[1]) == 1:
            options = [(a[0], b[1]), (b[0], a[1])]
            out.append(next((c for c in options if c in layer), min(options)))
    seen = set()
    return [c for c in out if not (c in seen or seen.add(c))]


def _borrow_colour(cells, cell):
    """Colour for an inserted cell: its nearest existing neighbour's, so a filled
    corner matches the line it joins instead of introducing a new colour."""
    x, y, z = cell
    for dx, dy in STEPS_4 + [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        colour = cells.get((x+dx, y+dy, z))
        if colour:
            return colour
    return next(iter(cells.values()))


def _walk(layer, start_cell):
    """Crack-following walk of one blob: the boundary cells, in walk order."""
    def filled(vertex, offset):
        return (vertex[0]+offset[0], vertex[1]+offset[1]) in layer

    def valid(vertex, direction):
        left, right = _SIDES[direction]
        return filled(vertex, left) and not filled(vertex, right)

    # The lowest-then-leftmost cell has empty space below it, so walking east
    # along its bottom edge satisfies the invariant.
    start = (start_cell[0], start_cell[1])
    state = (start, (1, 0))
    touched, order, seen = set(), [], set()
    while state not in seen:
        seen.add(state)
        vertex, direction = state
        cell = (vertex[0]+_SIDES[direction][0][0], vertex[1]+_SIDES[direction][0][1])
        if cell not in touched:
            touched.add(cell)
            order.append(cell)
        step = vertex[0]+direction[0], vertex[1]+direction[1]
        for turn in (_LEFT[direction], direction, _RIGHT[direction],
                     (-direction[0], -direction[1])):
            if valid(step, turn):
                state = (step, turn)
                break
        else:                              # a single isolated cell has no walk on
            break                          # from here; its own cell is recorded
    return order


def _enclosed(layer, walked):
    """Cells of this blob inside the walked boundary, found by flooding inward."""
    blob, frontier = set(walked), list(walked)
    while frontier:
        x, y = frontier.pop()
        for dx, dy in STEPS_4:
            cell = (x+dx, y+dy)
            if cell in layer and cell not in blob:
                blob.add(cell)
                frontier.append(cell)
    return blob


def connected(cells):
    """Largest 6-connected component, so the build is one object. Ties go to the
    component containing the lowest cell, which keeps the result deterministic."""
    if not cells:
        return {}
    unseen, components = set(cells), []
    while unseen:
        start = min(unseen)
        component, stack = {start}, [start]
        unseen.discard(start)
        while stack:
            for neighbour in _neighbours(stack.pop()):
                if neighbour in unseen:
                    unseen.discard(neighbour)
                    component.add(neighbour)
                    stack.append(neighbour)
        components.append(component)
    best = max(components, key=lambda c: (len(c), -min(c)[0], -min(c)[1]))
    return {cell: cells[cell] for cell in best}


def thin(cells):
    """Remove cubes until no 2x2 in-layer block is fully occupied.

    Every full 2x2 block contains a cube that cannot be gripped once its
    neighbours are down, so blocks are the direct cause of `no_clearance`.
    From each block the cube with the fewest occupied neighbours goes, because
    removing a low-degree cube is least likely to split the shape; if every
    candidate would split it, the lowest-degree one still goes and `connected`
    cleans up afterwards.
    """
    cells = dict(cells)
    while True:
        block = next((c for c in sorted(cells)
                      if all((c[0]+dx, c[1]+dy, c[2]) in cells
                             for dx, dy in [(1, 0), (0, 1), (1, 1)])), None)
        if block is None:
            return cells
        corners = [(block[0]+dx, block[1]+dy, block[2])
                   for dx, dy in [(0, 0), (1, 0), (0, 1), (1, 1)]]
        degree = {c: sum(n in cells for n in _neighbours(c)) for c in corners}
        keeps_shape = [c for c in corners
                       if len(connected({k: v for k, v in cells.items() if k != c}))
                       == len(cells)-1]
        candidates = keeps_shape or corners
        del cells[min(candidates, key=lambda c: (degree[c], c))]


def supported(cells):
    """Drop cubes with nothing directly below, lowest layer first, until every
    remaining cube rests on the table or on another cube."""
    cells = dict(cells)
    for cell in sorted(cells, key=lambda c: c[2]):
        if cell[2] and (cell[0], cell[1], cell[2]-1) not in cells:
            del cells[cell]
    return cells


def in_grid(cells, grid):
    if not grid:
        return dict(cells)
    return {c: v for c, v in cells.items() if 0 <= c[0] < grid[0] and 0 <= c[1] < grid[1]}


def placeable(cells):
    """Drop cells the placement search cannot reach with the fingers."""
    from schema import Voxel
    voxels = [Voxel(*cell, colour) for cell, colour in sorted(cells.items())]
    _, blocked = sequence(voxels)
    return {c: v for c, v in cells.items() if c not in set(blocked)}


def make_buildable(cells, grid=None, keep_outline=True, rounds=8):
    """Reduce a cell map until the buildability rules hold, or give up saying so.

    Returns `(cells, actions)`. `actions` lists `(what, [cells dropped])` in the
    order applied, so the caller can report the reduction. An empty result means
    nothing survived, which is a real answer for an image with no usable subject.
    """
    cells = dict(cells)
    actions = []

    def apply(reason, changed):
        nonlocal cells
        for action, difference in [('dropped', sorted(set(cells) - set(changed))),
                                   ('added', sorted(set(changed) - set(cells)))]:
            if difference:
                actions.append({'action': action, 'reason': reason, 'cells': difference})
        cells = changed

    apply('outside the build grid', in_grid(cells, grid))
    if keep_outline:
        apply('traced the outer contour of the silhouette, closing diagonal corners',
              contour(cells))
    for _ in range(rounds):
        before = len(cells)
        apply('part of a solid 2x2 block the gripper cannot finish', thin(cells))
        apply('unsupported (nothing directly below)', supported(cells))
        apply('not connected to the largest piece', connected(cells))
        apply('no finger clearance in any placement order', placeable(cells))
        if len(cells) == before:
            break
    return cells, actions


def report(actions):
    """One line per change, for a human to read before anything is built."""
    return '\n'.join(f'  {a["action"]} {len(a["cells"])} '
                     f'cube{"s" if len(a["cells"]) > 1 else ""}: {a["reason"]}'
                     for a in actions)


def summary(actions):
    """Compact record of the reduction, for a structure's `provenance` field."""
    return [f'{a["action"]} {len(a["cells"])}: {a["reason"]}' for a in actions]
