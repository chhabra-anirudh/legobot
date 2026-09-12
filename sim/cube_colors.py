"""Stable named cube colors, independent of whether a cube is held."""
COLORS = {'red': (225, 65, 65), 'green': (55, 180, 105), 'blue': (55, 130, 235),
          'yellow': (245, 200, 55), 'orange': (240, 125, 45), 'white': (235, 235, 235)}


def rgb(name):
    if not isinstance(name, str) or name not in COLORS:
        raise ValueError(f'unknown cube color {name!r}; choose from {list(COLORS)}')
    return COLORS[name]
