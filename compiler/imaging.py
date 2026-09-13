"""Image to a coarse colour grid, deterministically.

This is the front half of the picture pipeline. It reduces an image to one cell
per cube, at the resolution the arm can actually build, and names each cell's
colour from the cube palette in `sim/cube_colors.py`. Nothing here calls a model,
so the same file always produces the same grid, and the grid is reported next to
the picture so the reduction is visible rather than implied.

It deliberately decides nothing about buildability: a downsampled photograph is a
solid silhouette, which the gripper cannot finish. `simplify.py` reduces a grid to
something placeable and `schema.validate` decides whether the result is acceptable.
"""
import hashlib
import sys
from dataclasses import dataclass, field
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent/'sim'
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
from cube_colors import COLORS  # noqa: E402  single source of truth for colours

from preview import GLYPHS  # noqa: E402

# A cell counts as part of the subject when this much of it is foreground.
DEFAULT_COVERAGE = .45
# How far a pixel must sit from the estimated background colour, as a Euclidean
# distance in 0-255 RGB, before it counts as subject. Only used when the image
# has no usable alpha channel.
DEFAULT_TOLERANCE = 60.


def digest(path):
    """Content hash of the source image, so a structure records what it came from."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nearest_color(pixel):
    """Closest cube colour to an RGB triple. Euclidean in RGB, which is crude but
    predictable; the palette has six widely separated entries and no greys, so a
    dark subject lands on whichever hue it is nearest, not on black."""
    return min(COLORS, key=lambda name: sum((a-b)**2 for a, b in zip(COLORS[name], pixel)))


@dataclass
class Sheet:
    """A grid of cube cells read off an image. `cells` maps (x, y) to a colour name,
    with +x right and +y up, matching the voxel convention rather than image rows."""
    width: int
    height: int
    cells: dict = field(default_factory=dict)
    coverage: dict = field(default_factory=dict)
    image_path: str = ''
    image_sha256: str = ''

    def ascii(self):
        """Top view, first row highest y, using the same glyphs as the previewer."""
        return '\n'.join(''.join(GLYPHS.get(self.cells.get((x, y)), '.')
                                 for x in range(self.width))
                         for y in range(self.height-1, -1, -1))

    def counts(self):
        colours = {}
        for colour in self.cells.values():
            colours[colour] = colours.get(colour, 0) + 1
        return {'cells': len(self.cells), 'colors': dict(sorted(colours.items()))}

    def provenance(self):
        return {'image_path': self.image_path, 'image_sha256': self.image_sha256,
                'grid': [self.width, self.height]}


def _array(path):
    try:
        from PIL import Image
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise SystemExit('reading images needs Pillow; '
                         'pip install -r requirements-llm.txt') from exc
    import numpy as np
    if not Path(path).is_file():
        raise SystemExit(f'no such image: {path}')
    with Image.open(path) as handle:
        return np.asarray(handle.convert('RGBA'), dtype=float)


def foreground_mask(array, tolerance=DEFAULT_TOLERANCE):
    """Boolean subject mask. Alpha wins when the image has any transparency;
    otherwise the border pixels estimate the background and anything far enough
    from it is subject. Neither test is scene understanding: a photograph with a
    busy background will produce a busy mask, and it will show up in the preview."""
    import numpy as np
    alpha, rgb = array[..., 3], array[..., :3]
    if alpha.min() < 250:
        return alpha > 127
    border = np.concatenate([rgb[0], rgb[-1], rgb[:, 0], rgb[:, -1]])
    background = np.median(border, axis=0)
    return np.sqrt(((rgb-background)**2).sum(axis=-1)) > tolerance


def to_sheet(path, grid=(10, 8), coverage=DEFAULT_COVERAGE, mono=None,
             tolerance=DEFAULT_TOLERANCE, invert=False):
    """Downsample an image onto a `grid` of cube cells.

    Aspect ratio is preserved and the image is centred in the grid, so a wide
    picture leaves the top and bottom rows empty instead of being squashed. Pass
    `mono` to force one cube colour, which is the honest choice when the real
    inventory of a colour is not known.
    """
    import numpy as np
    if mono is not None and mono not in COLORS:
        raise ValueError(f'unknown cube color {mono!r}; choose from {sorted(COLORS)}')
    width, height = int(grid[0]), int(grid[1])
    if width < 1 or height < 1:
        raise ValueError(f'grid must be at least 1x1, got {width}x{height}')
    array = _array(path)
    mask = foreground_mask(array, tolerance)
    if invert:
        mask = ~mask
    rows, columns = mask.shape
    step = max(columns/width, rows/height)
    x0 = (width*step - columns)/2       # centre the image inside the grid
    y0 = (height*step - rows)/2

    sheet = Sheet(width, height, image_path=str(path), image_sha256=digest(path))
    for gx in range(width):
        for row in range(height):
            left, right = int(round(gx*step - x0)), int(round((gx+1)*step - x0))
            top, bottom = int(round(row*step - y0)), int(round((row+1)*step - y0))
            window = mask[max(top, 0):max(bottom, 0), max(left, 0):max(right, 0)]
            if not window.size:
                continue
            # Cells partly outside the image are judged on their whole area, so a
            # sliver of subject at the edge does not become a cube.
            area = max((bottom-top)*(right-left), 1)
            filled = float(window.sum())/area
            y = height-1-row                      # image rows run downward, y runs up
            sheet.coverage[(gx, y)] = filled
            if filled < coverage:
                continue
            pixels = array[max(top, 0):max(bottom, 0), max(left, 0):max(right, 0), :3]
            mean = pixels[window].mean(axis=0) if window.any() else pixels.mean(axis=(0, 1))
            sheet.cells[(gx, y)] = mono or nearest_color(tuple(mean))
    return sheet
