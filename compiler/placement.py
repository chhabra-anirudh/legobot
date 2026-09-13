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


def sequence(voxels):
    """Bottom-up order respecting direct support and finger clearance.

    Greedy and most-constrained-first: a cube with many neighbours in the
    finished structure runs out of clearance soonest, so it goes early.

    Returns (steps, blocked) where steps is a list of (voxel, axis). A non-empty
    `blocked` means this heuristic found no order — evidence that the structure
    is impractical, not a proof that no order exists.
    """
    remaining = {v.cell: v for v in voxels}

    def on_axis(cell, axis):
        return sum((cell[0]+dx, cell[1]+dy, cell[2]+dz) in remaining
                   for dx, dy, dz in NEIGHBOURS[axis])

    degree = {cell: on_axis(cell, 'x') + on_axis(cell, 'y') for cell in remaining}
    # A junction has neighbours on both axes in the finished structure, so once
    # any of them is down it can never be gripped. Junctions go first, while the
    # cells around them are still empty.
    junction = {cell: bool(on_axis(cell, 'x')) and bool(on_axis(cell, 'y'))
                for cell in remaining}
    placed, steps = set(), []
    while remaining:
        ready = []
        for cell in remaining:
            if cell[2] and (cell[0], cell[1], cell[2]-1) not in placed:
                continue  # support is not there yet
            axis = free_axis(cell, placed)
            if axis:
                ready.append((cell, axis))
        if not ready:
            return steps, sorted(remaining)
        cell, axis = min(ready, key=lambda ca: (ca[0][2], not junction[ca[0]],
                                                -degree[ca[0]], ca[0][1], ca[0][0]))
        steps.append((remaining.pop(cell), axis))
        placed.add(cell)
    return steps, []


def clearance_problems(voxels):
    """Cells the placement search could not reach, as (cell, reason) pairs."""
    _, blocked = sequence(voxels)
    return [(cell, 'no finger clearance at any point in the placement order')
            for cell in blocked]
