"""Prompt to voxel model, with a deterministic check after every proposal.

The model proposes; `schema.validate` decides. A proposal is never trusted
because it came back well-formed: the same checks run on model output and on
offline library shapes. Structures carry the source that produced them so a
hand-authored shape is never reported as model output.
"""
import argparse
import json
import sys
from pathlib import Path

from schema import (COLORS, DEFAULT_LIMITS, NOMINAL_CUBE_SIZE_M, Structure, Voxel,
                    build_order, save, validate)
from preview import render

MODEL = 'claude-opus-5'

SYSTEM = """You design small voxel sculptures that a robot arm builds from 25.4 mm
magnetic cubes, one cube at a time.

Hard rules, enforced by a deterministic checker after you answer:
- Integer grid coordinates. x and y are the build surface; z is height. z=0 rests
  on the table.
- Direct support only: every cube with z>0 needs a cube in the cell directly
  below it (same x, same y, z-1). Overhangs, bridges, and floating cubes are
  rejected. A cube resting only on diagonal or side neighbours is rejected.
- One connected object: every cube must reach every other through shared faces.
- Colors must come from this palette: {colors}.
- At most {max_cubes} cubes and {max_layers} layers.
- The reachable build area is a grid {grid_w} cells wide (x from 0 to {grid_x}) by
  {grid_h} cells deep (y from 0 to {grid_y}). Every cube must be inside it. This is
  the arm's measured reach, not a stylistic limit — a design that spills outside it
  cannot be built at all.

- Finger clearance: the gripper holds a cube by two opposite faces from above, so
  each pad needs the whole neighbouring cell. A cube can only be placed while both
  cells along one horizontal axis are still empty. The consequence is strict: any
  solid region two or more cells wide in BOTH x and y is impossible to finish,
  because the cube placed last in it is boxed in on every side.

Two rules therefore shape every design. Overhangs are rejected, so a standing
figure with legs and a raised body is not buildable. Solid areas are rejected, so
filled-in shapes are not buildable either.

What works is line art: a path of cubes ONE CELL WIDE, lying flat on the table at
z=0, drawn as an outline of the subject. Diagonal steps are fine as long as each
cube shares a face with the next. Avoid placing two cubes side by side unless the
line genuinely runs that way, and never fill an interior.

Draw the requested subject as a recognisable one-cube-wide outline. Use the grid
generously; a bigger outline reads better than a few cubes."""

SCHEMA = {
    'type': 'object',
    'properties': {
        'name': {'type': 'string', 'description': 'short name for the structure'},
        'reasoning': {'type': 'string',
                      'description': 'one or two sentences on the shape and how it is supported'},
        'voxels': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {'x': {'type': 'integer'}, 'y': {'type': 'integer'},
                               'z': {'type': 'integer'},
                               'color': {'type': 'string', 'enum': sorted(COLORS)}},
                'required': ['x', 'y', 'z', 'color'],
                'additionalProperties': False,
            },
        },
    },
    'required': ['name', 'reasoning', 'voxels'],
    'additionalProperties': False,
}

# Hand-authored fallbacks for running the pipeline without model access. These
# are not model output and are labelled 'offline_library' wherever they appear.
LIBRARY = {
    'dog': ("""
.......OOOOO
.......O...O
OOOOOOOO...R
O..........O
O.......OOOO
OOOOOOOO....
..O....O....
..O....O....
..W....W....
""", 'one-cube-wide dog outline, side profile facing right'),
}
GLYPH_COLORS = {'O': 'orange', 'W': 'white', 'R': 'red', 'G': 'green', 'B': 'blue',
                'Y': 'yellow'}


def from_ascii(art, z=0):
    """Parse an ASCII layer into voxels. The first row is the highest y."""
    rows = [r for r in art.strip('\n').splitlines() if r.strip()]
    height = len(rows)
    voxels = []
    for row_index, row in enumerate(rows):
        for x, glyph in enumerate(row):
            if glyph == '.':
                continue
            if glyph not in GLYPH_COLORS:
                raise ValueError(f'unknown glyph {glyph!r}; use {sorted(GLYPH_COLORS)} or "."')
            voxels.append(Voxel(x, height-1-row_index, z, GLYPH_COLORS[glyph]))
    return voxels


def offline(subject, structure_id=None):
    """Return a library structure. Never labelled as model output."""
    key = subject.strip().lower()
    match = next((k for k in LIBRARY if k in key), None)
    if match is None:
        raise KeyError(f'no offline shape for {subject!r}; available: {sorted(LIBRARY)}')
    art, description = LIBRARY[match]
    return Structure(structure_id=structure_id or f'{match}-offline', name=description,
                     voxels=from_ascii(art), source='offline_library', prompt=subject)


def _client():
    try:
        import anthropic
    except ImportError as exc:
        raise SystemExit('the anthropic SDK is not installed; '
                         'pip install -r requirements-llm.txt') from exc
    try:
        return anthropic.Anthropic()
    except Exception as exc:  # missing or unreadable credentials
        raise SystemExit(f'could not construct an Anthropic client: {exc}\n'
                         'Set ANTHROPIC_API_KEY, or run `ant auth login`, then retry. '
                         'Use --offline to run the checker without model access.') from exc


def system_prompt(limits=None, guidance=''):
    """The rules the model is asked to design under, for a given set of limits."""
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    grid = limits.get('grid') or (99, 99)
    text = SYSTEM.format(colors=', '.join(sorted(COLORS)), max_cubes=limits['max_cubes'],
                         max_layers=limits['max_layers'], grid_w=grid[0], grid_h=grid[1],
                         grid_x=grid[0]-1, grid_y=grid[1]-1)
    return f'{text}\n\n{guidance}' if guidance else text


def propose(prompt, limits=None, attempts=3, model=MODEL, structure_id=None, verbose=False,
            attachments=None, guidance=''):
    """Ask the model for a structure, re-asking with the checker's problems.

    `attachments` are extra content blocks for the first user message, which is
    how an image reaches the model; `guidance` is appended to the system rules.
    Returns (structure, report, transcript). The structure is returned even when
    it fails, so the caller can show why it was rejected.
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    client = _client()
    system = system_prompt(limits, guidance)
    messages = [{'role': 'user',
                 'content': [{'type': 'text', 'text': f'Design a voxel structure: {prompt}'}]
                            + list(attachments or [])}]
    transcript, structure, report = [], None, None
    for attempt in range(1, attempts+1):
        response = client.messages.create(
            model=model,
            max_tokens=16000,
            system=system,
            messages=messages,
            thinking={'type': 'adaptive'},
            output_config={'format': {'type': 'json_schema', 'schema': SCHEMA}},
        )
        if response.stop_reason == 'refusal':
            raise SystemExit(f'model declined the request: {response.stop_details}')
        text = ''.join(b.text for b in response.content if b.type == 'text')
        proposal = json.loads(text)
        structure = Structure(
            structure_id=structure_id or f'{prompt[:24].strip().replace(" ", "-").lower()}-{attempt}',
            name=proposal.get('name', prompt), voxels=[Voxel(**v) for v in proposal['voxels']],
            cube_size_m=NOMINAL_CUBE_SIZE_M, source=model, prompt=prompt)
        report = validate(structure, limits)
        transcript.append({'attempt': attempt, 'reasoning': proposal.get('reasoning', ''),
                           'cubes': len(structure.voxels),
                           'problems': [str(p) for p in report.problems]})
        if verbose:
            print(f'attempt {attempt}: {len(structure.voxels)} cubes, '
                  f'{len(report.problems)} problems', file=sys.stderr)
        if report.ok or attempt == attempts:
            return structure, report, transcript
        # Repair round: the model sees its own output and every failure, and the
        # full check runs again on whatever comes back.
        problems = '\n'.join(f'- {p}' for p in report.problems[:40])
        messages += [{'role': 'assistant', 'content': text},
                     {'role': 'user', 'content': f'The checker rejected that structure:\n'
                                                 f'{problems}\n\nFix every problem and return '
                                                 f'the corrected structure. Keep it recognisable '
                                                 f'as: {prompt}'}]
    return structure, report, transcript


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('prompt', help='what to build, e.g. "a simple dog"')
    parser.add_argument('--offline', action='store_true',
                        help='use the hand-authored library instead of calling the model')
    parser.add_argument('--model', default=MODEL)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--max-cubes', type=int, default=DEFAULT_LIMITS['max_cubes'])
    parser.add_argument('--max-layers', type=int, default=DEFAULT_LIMITS['max_layers'])
    parser.add_argument('--grid', type=int, nargs=2, default=[7, 5], metavar=('W', 'H'),
                        help='reachable build area in cells')
    parser.add_argument('--output', help='write the accepted structure JSON here')
    parser.add_argument('--order', action='store_true', help='print the bottom-up build order')
    args = parser.parse_args(argv)

    limits = {**DEFAULT_LIMITS, 'max_cubes': args.max_cubes,
              'max_layers': args.max_layers, 'grid': tuple(args.grid)}
    if args.offline:
        structure = offline(args.prompt)
        report = validate(structure, limits)
    else:
        structure, report, _ = propose(args.prompt, limits, args.attempts, args.model,
                                       verbose=True)
    print(render(structure, report))
    if args.order:
        print('\nbuild order (placement sequence only; no tool poses):')
        for i, v in enumerate(build_order(structure), 1):
            print(f'  {i:3d}. ({v.x}, {v.y}, {v.z}) {v.color}')
    if args.output and report.ok:
        save(structure, args.output)
        print(f'\nwrote {args.output}')
    elif args.output:
        print('\nnot written: the structure was rejected', file=sys.stderr)
    return 0 if report.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
