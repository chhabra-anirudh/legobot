# /// script
# dependencies = ["bbos", "numpy<2"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Read-only: drain a daemon's log channel."""
import sys, time
from bbos import Reader
name = sys.argv[1] if len(sys.argv) > 1 else 'arm_left.log'
with Reader(name) as r:
    end = time.time() + 8
    seen = 0
    while time.time() < end:
        if r.ready():
            d = r.data
            if d is not None:
                msg = d['msg'] if 'msg' in getattr(d, 'dtype', d).names else d
                text = bytes(msg).decode('utf-8', 'replace').strip('\x00').strip()
                if text:
                    print(text); seen += 1
        time.sleep(.05)
    print(f'({seen} log records)')
