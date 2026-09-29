"""Build every figure of the book (or those whose name contains the argument).

Usage: python3 book/tools/make_figures.py [filter]
"""

import sys
import time
import traceback
from multiprocessing import Pool

import figlib
import figs_theory  # noqa: F401  (registers figures)
import figs_part1  # noqa: F401
import figs_part2  # noqa: F401


def build(name):
    t0 = time.time()
    try:
        figlib.REGISTRY[name]()
        return f"{name:28s} {time.time() - t0:5.1f} s"
    except Exception:
        return f"{name:28s} FAILED\n{traceback.format_exc()}"


if __name__ == "__main__":
    flt = sys.argv[1] if len(sys.argv) > 1 else ""
    names = [n for n in figlib.REGISTRY if flt in n]
    with Pool(8) as pool:
        for line in pool.imap_unordered(build, names):
            print(line, flush=True)
