# Prompt or picture to a checked voxel structure

```
prompt ─┐
        ├─> LLM -> voxel schema -> deterministic checker -> preview -> sim/build_structure.py
image ──┘
```

The model proposes a structure; `schema.py` decides whether it is acceptable.
The checker never calls a model, and model output is never trusted because it
came back well formed. Finger clearance and placement order are checked here;
robot tool poses and reachability are decided by `sim/build_structure.py`, which
takes the structure files this directory produces.

For the whole demo in one command — description, checks, and the simulated build —
use `build_from_description.py` at the repository root. This directory is the
compiler half of it.

## Run it

```sh
.venv/bin/python -m pip install -r requirements-llm.txt   # anthropic SDK + Pillow
.venv/bin/python compiler/generate.py "a simple dog" --output outputs/dog.json
```

Credentials come from `ANTHROPIC_API_KEY` or an `ant auth login` profile. Without
model access, run the hand-authored library shape through the identical checker:

```sh
.venv/bin/python compiler/generate.py "a simple dog" --offline
```

Check a saved structure, or print the placement order:

```sh
.venv/bin/python compiler/check.py compiler/examples/dog.json --order
.venv/bin/python -m unittest discover -s compiler -p 'test_*.py'
```

## From a picture

```sh
# deterministic: no model anywhere in the path
.venv/bin/python compiler/image_to_structure.py examples/images/cat.png --trace \
    --grid 12 8 --output outputs/cat.json

# with a vision model: the image and the downsample both go to the model
.venv/bin/python compiler/image_to_structure.py photo.png --subject "a cat" \
    --grid 12 8 --output outputs/cat.json
```

`imaging.py` downsamples the image onto the build grid — aspect ratio preserved,
image centred, each cell's colour snapped to the cube palette — and prints the
result, so the reduction is visible instead of implied. Subject and background are
separated by alpha when the image has any, otherwise by distance from a background
colour estimated from the border pixels. That works on drawings, logos, and
subjects on a plain background; a busy photograph produces a busy mask, which is
why `--tolerance`, `--coverage`, and `--invert` exist and why the model path sends
the **real image** as well as the downsample.

**A picture is never buildable as a picture.** A silhouette is solid, and a solid
region two or more cells wide in both x and y can never be finished: the cube
placed last in it has no free side for a finger pad. `simplify.py` therefore
reduces a grid to line art — it traces the outer contour of the silhouette, fills
in diagonal corners so the loop stays face-connected, thins any remaining 2x2
block, drops unsupported and stranded cubes, and repeats until `schema.validate`
accepts the result. Every cube it drops or adds is printed and recorded in the
structure's `provenance`, so a simplification is never presented as the picture.

Three paths, and each records honestly which one produced the structure:

| Path | Source label | Model access |
| --- | --- | --- |
| `--trace` | `image_trace` | none; fully deterministic |
| default | the model id | vision model, called by this pipeline |
| `--request` / `--ingest` | whatever `--source` states | an external model or agent answers |

`--request FILE` writes the exact request (rules, downsampled grid, response
schema) for something else to answer; `--ingest REPLY --source LABEL` reads the
answer back and runs the identical checks on it. That exists so an agent can act
as the vision model without the pipeline pretending it called an API.

`request.py` holds that plumbing, and the prompt path in `build_from_description.py`
uses the same three options with the same file format.

## What gets checked

Every proposal runs through the same rules, in `schema.validate`:

| Code | Rejected because |
| --- | --- |
| `non_integer`, `negative_layer` | coordinates are not integers, or sit below the table |
| `duplicate` | two cubes in one cell |
| `unknown_color` | color outside `sim/cube_colors.py` |
| `unsupported` | no cube directly below; overhangs and bridges are excluded |
| `disconnected` | cubes not reachable through shared faces |
| `no_clearance` | no placement order leaves a free cell for a finger pad |
| `out_of_grid` | outside the reachable build area passed as `grid` |
| `cube_budget`, `layer_budget`, `footprint`, `inventory` | exceeds the stated limits |
| `cube_size`, `schema_version`, `missing_id`, `empty` | schema violations |

Support is **direct support only**. A cube resting on diagonal or side
neighbours is rejected. This is why a standing figure with legs and a raised
body is not buildable, and why recognisable subjects come out as flat
silhouettes at `z=0` — every cube then rests on the table.

When a proposal fails, `generate.py` sends the model its own output plus every
problem and re-runs the **full** check on the reply, up to `--attempts` times. A
repair is never accepted without re-checking; fixing one voxel can break support
or connectivity elsewhere.

`no_clearance` deserves a note, because it used to fire on structures that were
fine. `placement.py` ranks cubes and places the most constrained first, which is
fast but not complete: a corner with one neighbour on each axis can be walled in
by two junctions that both outranked it. A failed order is now retried with the
stranded cubes forced to the front, up to `RETRIES` times. This recovered two
cubes of the cat below. A structure still reported as blocked is evidence that it
is impractical, not proof that no order exists.

## Cached examples

`examples/dog.json` — 16 cubes, one layer, generated by `claude-opus-5` from the
prompt "a simple dog" and accepted on the first attempt:

```
.WOOOOOY
WW.O..O.
W.WO..OW
```

`examples/cat.json` — 36 cubes, one layer, from `examples/images/cat.png` (a flat
colour drawing made with Pillow for this example, not a photograph):

```
Y.......O.O
Y......OOOO
YOOOOOOO..O
.O.....O..R
.O.....OOOO
.OOOOOOO...
..O...O....
..W...W....
```

Yellow tail, red nose, white paws, orange body, viewed from above with `+x` right
and `+y` up. Its `source` records that it came from a vision reply produced by an
interactive `claude-opus-5-medium` agent session rather than by an API call from
this pipeline, and its `provenance` records the image path, the image's SHA-256,
and the grid it was designed on.

`examples/house.json` — 40 cubes, one layer, from the **description** "a small
house" rather than from a picture, answered by an interactive agent session:

```
..RRRRR..
.RR...RR.
.R.....R.
WWWWWWWWW
W.......W
W......YW
W...B...W
W...B...W
WWWWWWWWW
```

Hollow white walls, a stepped red roof, a blue door, a yellow window. It is the
largest example that plans cleanly: 40 cubes, 3991 joint-limited poses, and 48 is
the measured limit on isolated staging slots beside a build at the 0.5 m table.

`examples/dog-floating-rejected.json` — the dog with one cube lifted to `z=2` and
one cube set apart. Hand-edited as a negative control, and labelled as such in its
`source` field. It is rejected for `unsupported`, `disconnected`, and
`no_clearance`.

Each structure records the `source` that produced it: a model id, or
`offline_library`, or `image_trace`, or `hand_edited_from_*`. A hand-authored or
deterministically traced shape is never reported as model output.

## Limits

The grid is dimensionless here. `DEFAULT_LIMITS` (64 cubes, 4 layers, 12x12
footprint) and the per-color inventory are **provisional** — real cube stock and a
validated layer count are not confirmed. The checker establishes validity under
these rules only. It says nothing about friction, calibration, magnetic behavior,
or whether a build will actually succeed.

What the grid can be is measured, though, by the arm and not by this directory:
at the 0.5 m table, `12x8` is about the largest build area that fits the reach
map, and 36 cubes still find isolated staging slots beside it. Check a specific
case with `sim/build_structure.py <structure> --list-origins`.

The vision-model path (`image_to_structure.py` with no `--trace`/`--ingest`) is
implemented but has **not** been run against a live API in this checkout: there
were no credentials available. `--trace` and `--ingest` are exercised by the tests.
