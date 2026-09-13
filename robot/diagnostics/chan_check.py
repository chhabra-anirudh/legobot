# /// script
# dependencies = ["bbos", "numpy<2"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Read-only: which IPC channels actually have a live writer right now."""
import time
from bbos import Reader, Type

CHANNELS = [('arm_left.state', 'arm_state'), ('arm_right.state', 'arm_state'),
            ('arm_left.ctrl', 'arm_ctrl'), ('base.health', None),
            ('camera.head.status', None), ('telemetry.state', None)]
for name, typename in CHANNELS:
    try:
        r = Reader(name, Type(typename)) if typename else Reader(name)
        with r:
            deadline = time.time() + 2
            while not r.ready() and time.time() < deadline:
                time.sleep(.05)
            print(f'{name:22s} readable={r.ready()}')
    except Exception as exc:
        print(f'{name:22s} error: {type(exc).__name__}: {exc}')
