"""The request a model answers, and the reply it sends back.

Shared by the prompt path and the picture path, so both write the same request
file, read the same reply, and run the identical checks on it. It exists because
model access is not always available: the request can be answered by an API call,
or written to a file for something else — another model, or an agent in a session —
to answer, without this code ever claiming which one it was. The caller must state
a `source`; nothing here invents one.
"""
import json
from pathlib import Path

import simplify
from generate import SCHEMA, system_prompt
from schema import Structure, Voxel

REPLY_INSTRUCTIONS = ('Reply with a JSON object matching response_schema. Save it to a '
                      'file and pass it back with --ingest FILE --source <what answered '
                      'it>, which runs the full deterministic check on it.')


def write(path, prompt, limits, guidance='', extra=None):
    """Write the request: the rules, what to design, and the reply schema."""
    request = {'system': system_prompt(limits, guidance),
               'user_text': prompt,
               'limits': {'max_cubes': limits['max_cubes'], 'max_layers': limits['max_layers'],
                          'grid': list(limits['grid']) if limits.get('grid') else None},
               'response_schema': SCHEMA,
               'reply_with': REPLY_INSTRUCTIONS,
               **(extra or {})}
    Path(path).write_text(json.dumps(request, indent=2)+'\n')
    return request


def read(path):
    """Load a reply, failing loudly rather than half-building something."""
    reply = json.loads(Path(path).read_text())
    if not isinstance(reply, dict) or not isinstance(reply.get('voxels'), list):
        raise SystemExit(f'{path} must be a JSON object with a "voxels" list, matching '
                         'the response_schema in the request')
    for index, voxel in enumerate(reply['voxels']):
        missing = {'x', 'y', 'z', 'color'} - set(voxel or {})
        if missing:
            raise SystemExit(f'{path}: voxel {index} is missing {sorted(missing)}')
    return reply


def structure(reply, structure_id, source, prompt='', provenance=None, grid=None,
              reduce=True, keep_outline=False):
    """Turn a reply into a structure, optionally reducing it to something placeable.

    Returns `(structure, actions)`. `actions` is empty when nothing had to change.
    The reduction is the same deterministic one the picture path uses; it is
    reported by the caller, never hidden.
    """
    if not str(source).strip():
        raise SystemExit('a reply needs a source: record what produced it')
    cells = {(v['x'], v['y'], v['z']): v['color'] for v in reply['voxels']}
    actions = []
    if reduce:
        cells, actions = simplify.make_buildable(cells, grid=grid, keep_outline=keep_outline)
    record = {**(provenance or {}), 'reasoning': reply.get('reasoning', ''),
              'simplified': simplify.summary(actions)}
    return Structure(structure_id=structure_id, name=reply.get('name') or prompt or 'structure',
                     voxels=[Voxel(*cell, colour) for cell, colour in sorted(cells.items())],
                     source=str(source).strip(), prompt=prompt, provenance=record), actions
