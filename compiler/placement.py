"""Placement order and finger clearance.

The gripper has two pads on opposite faces of the cube, approaching from above.
A pad needs the whole neighbouring cell: if a cube is already sitting next to the
target cell, the pad on that side has nowhere to go. So a cube can only be placed
while both cells along one horizontal axis are still empty.

That rule has a consequence worth stating plainly: a region two or more cells
wide in both x and y cannot be completed, because the cube placed last in it has
occupied neighbours on both axes. Structures must be one cube wide, or stacked.

Pure python on purpose — the compiler decides order and clearance, the executor
turns the result into tool poses. Cube geometry lives here, arm geometry does not.
"""

NEIGHBOURS = {'x': [(1, 0, 0), (-1, 0, 0)], 'y': [(0, 1, 0), (0, -1, 0)]}


def free_axis(cell, placed):
    """Axis the fingers can close along, or None if both are obstructed.

    Only cubes already placed obstruct. Cubes that belong to the structure but
    are not placed yet do not: the fingers retract before the next placement.
    """
    for axis, steps in NEIGHBOURS.items():
        if all((cell[0]+dx, cell[1]+dy, cell[2]+dz) not in placed for dx, dy, dz in steps):
            return axis
    return None


# How many times a failed order is retried with the stranded cubes forced first.
# A cap, not a claim: hitting it means "not resolved", not "impossible".
RETRIES = 8


def _ranking(cells):
    """Order candidates are tried in: lowest layer, then junctions, then the cube
    with the most neighbours in the finished structure. A junction has neighbours
    on both axes, so once any of them is down it can never be gripped."""
    def on_axis(cell, axis):
        return sum((cell[0]+dx, cell[1]+dy, cell[2]+dz) in cells
                   for dx, dy, dz in NEIGHBOURS[axis])

    degree = {cell: on_axis(cell, 'x') + on_axis(cell, 'y') for cell in cells}
    junction = {cell: bool(on_axis(cell, 'x')) and bool(on_axis(cell, 'y'))
                for cell in cells}
    return lambda ca: (ca[0][2], not junction[ca[0]], -degree[ca[0]], ca[0][1], ca[0][0])


def _ready(cells, placed):
    """Cells that can be placed next, with the axis the fingers would close along."""
    out = []
    for cell in cells:
        if cell in placed:
            continue
        if cell[2] and (cell[0], cell[1], cell[2]-1) not in placed:
            continue                       # support is not there yet
        axis = free_axis(cell, placed)
        if axis:
            out.append((cell, axis))
    return out


def _greedy(cells, rank, first=()):
    """Take the highest-ranked placeable cube at every step, never reconsidering.

    Cells in `first` jump the queue as soon as their support is down. That is what
    makes the retry in `sequence` work: a cube stranded by the plain ranking is
    usually placeable if it simply goes earlier.
    """
    first, placed, order = set(first), set(), []
    while len(order) < len(cells):
        ready = _ready(cells, placed)
        if not ready:
            return order, sorted(set(cells) - placed)
        urgent = [ca for ca in ready if ca[0] in first]
        cell, axis = min(urgent or ready, key=rank)
        order.append((cell, axis))
        placed.add(cell)
    return order, []


def sequence(voxels, retries=RETRIES):
    """Bottom-up order respecting direct support and finger clearance.

    Ranked greedy first, because it is fast and usually right. Greedy is not
    complete, though: a corner with one neighbour on each axis can be walled in by
    two junctions that both outranked it, and it is then reported as having no
    clearance even though placing it first would have worked. Dropping a cube that
    was actually placeable quietly degrades every design, so a failed order is
    retried with the stranded cubes moved to the front, up to `retries` times.

    Returns (steps, blocked) where steps is a list of (voxel, axis). A non-empty
    `blocked` means no attempt found an order — evidence that the structure is
    impractical, not a proof that no order exists.
    """
    cells = {v.cell: v for v in voxels}
    rank = _ranking(cells)
    order, blocked = _greedy(cells, rank)
    forced = set()
    for _ in range(retries):
        if not blocked:
            break
        if forced.issuperset(blocked):
            break                          # the same cubes strand again; give up
        forced |= set(blocked)
        order, blocked = _greedy(cells, rank, forced)
    return [(cells[cell], axis) for cell, axis in order], blocked


def clearance_problems(voxels):
    """Cells the placement search could not reach, as (cell, reason) pairs."""
    _, blocked = sequence(voxels)
    return [(cell, 'no finger clearance at any point in the placement order')
            for cell in blocked]
