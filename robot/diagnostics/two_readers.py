# /// script
# dependencies = ["bbos", "numpy<2"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Does the first Reader in a process warm up slower than later ones?"""
import time
import numpy as np
from bbos import Reader, Type

for attempt in (1, 2, 3):
    with Reader('arm_left.state', Type('arm_state')) as r:
        end = time.time() + 3
        while not r.ready() and time.time() < end:
            time.sleep(.05)
        ok = r.ready()
        pos = np.asarray(r.data['pos']).round(4).tolist() if ok else None
        print(f'attempt {attempt}: readable={ok} pos={pos}')
