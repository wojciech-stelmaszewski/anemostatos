"""Shared plotting style for the book: the same fonts as the text (via the PGF/LuaLaTeX backend)
and the same colours as the simulator (src/ui/colors.ts)."""

import functools
from pathlib import Path

import matplotlib

matplotlib.use("pgf")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
OUT = ROOT / "figures" / "out"
OUT.mkdir(parents=True, exist_ok=True)

MM = 1 / 25.4
TEXT_W = 124 * MM
FULL_W = 177 * MM
MARGIN_W = 46 * MM

C = dict(
    ink="#1C2330", muted="#5D6675", faint="#8A93A3", hair="#D9DDE3", mist="#F3F5F8",
    accent="#E8661A", night="#0B0E13", blue="#2F5D8A",
    sp="#8A93A3", meas="#3987E5", err="#D64F4F", P="#D95926", I="#199E70", D="#7C70DA",
    FF="#C98500", wind="#2DA44E", truth="#1C2330", ghost="#B8BEC8", sat="#F6D5D0",
    m1="#3987E5", m2="#D95926", m3="#199E70", m4="#C98500",
)
# an ordered, colour-blind-checked sequence for "several variants of one thing"
SEQ = ["#3987E5", "#D95926", "#199E70", "#7C70DA", "#C98500", "#D64F4F", "#1C2330"]
# a light→dark ramp in the accent hue for parameter sweeps
def ramp(n, hue="accent"):
    base = {"accent": (232, 102, 26), "blue": (57, 135, 229), "ink": (28, 35, 48)}[hue]
    out = []
    for k in range(n):
        f = 0.28 + 0.72 * (k / max(n - 1, 1))
        out.append(tuple((255 - (255 - c) * f) / 255 for c in base))
    return out


plt.rcParams.update({
    "pgf.texsystem": "lualatex",
    "pgf.rcfonts": False,
    "pgf.preamble": "\n".join([
        r"\usepackage{fontspec}",
        r"\usepackage{unicode-math}",
        r"\setmainfont{FiraSans}[Extension=.otf,UprightFont=*-Book,ItalicFont=*-BookItalic,BoldFont=*-Medium,Numbers=Lining]",
        r"\setsansfont{FiraSans}[Extension=.otf,UprightFont=*-Book,ItalicFont=*-BookItalic,BoldFont=*-Medium,Numbers=Lining]",
        r"\setmonofont{FiraMono}[Extension=.otf,UprightFont=*-Regular,Scale=0.86]",
        r"\setmathfont{LibertinusMath-Regular.otf}",
    ]),
    "font.family": "sans-serif",
    "font.size": 7.6,
    "axes.titlesize": 8.2,
    "axes.labelsize": 7.6,
    "xtick.labelsize": 6.8,
    "ytick.labelsize": 6.8,
    "legend.fontsize": 6.8,
    "axes.edgecolor": C["faint"],
    "axes.labelcolor": C["muted"],
    "axes.linewidth": 0.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "axes.grid.axis": "y",
    "grid.color": "#E6E9EE",
    "grid.linewidth": 0.45,
    "xtick.color": C["faint"],
    "ytick.color": C["faint"],
    "xtick.major.size": 2.5,
    "ytick.major.size": 0,
    "xtick.major.width": 0.5,
    "lines.linewidth": 1.15,
    "lines.solid_capstyle": "round",
    "legend.frameon": False,
    "legend.handlelength": 1.6,
    "axes.titlelocation": "left",
    "axes.titlepad": 5,
    "axes.titleweight": "bold",
    "axes.titlecolor": C["ink"],
    "figure.dpi": 150,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "axes.prop_cycle": matplotlib.cycler(color=SEQ),
})

REGISTRY = {}


def fig(fn):
    """Register a figure builder; its name is the output file name."""
    REGISTRY[fn.__name__] = fn
    return fn


@functools.lru_cache(maxsize=None)
def load(name):
    return pd.read_csv(DATA / f"{name}.csv.gz")


def win(d, t0, t1):
    return d[(d.t >= t0) & (d.t <= t1)]


def save(f, name):
    f.savefig(OUT / f"{name}.pdf")
    plt.close(f)


def label_end(ax, x, y, text, color, dx=0.0, dy=0.0, ha="left", va="center", size=6.8, **kw):
    ax.annotate(text, (x, y), xytext=(x + dx, y + dy), color=color, fontsize=size, ha=ha, va=va,
                annotation_clip=False, **kw)


def note(ax, x, y, text, xt, yt, color=None, ha="left", va="center", size=6.6, rad=0.2):
    color = color or C["muted"]
    ax.annotate(text, (x, y), xytext=(xt, yt), fontsize=size, color=color, ha=ha, va=va,
                annotation_clip=False, arrowprops=dict(arrowstyle="-|>", color=color, lw=0.5, mutation_scale=5,
                                shrinkA=1, shrinkB=1.5, connectionstyle=f"arc3,rad={rad}"))


def sat_shade(ax, t, sat, color=None):
    """Shade intervals where a saturation flag is 1 (as the app's red shading)."""
    color = color or C["sat"]
    s = np.asarray(sat) > 0.5
    t = np.asarray(t)
    start = None
    for k in range(len(s)):
        if s[k] and start is None:
            start = t[k]
        if (not s[k] or k == len(s) - 1) and start is not None:
            ax.axvspan(start, t[k], color=color, lw=0, zorder=0)
            start = None


def xlab_time(ax, text="time  [s]"):
    ax.set_xlabel(text, loc="right")


def panel_title(ax, text, sub=None):
    ax.set_title(text)
    if sub:
        ax.text(0, 1.0, "", transform=ax.transAxes)


def step_metrics(t, y, t0, y0, y1):
    """Rise time (10→90 %), overshoot (%), 2 % settling time after a step at t0."""
    t = np.asarray(t)
    y = np.asarray(y)
    m = t >= t0
    t, y = t[m], y[m]
    s = y1 - y0
    f = (y - y0) / s
    try:
        tr = t[np.argmax(f >= 0.9)] - t[np.argmax(f >= 0.1)]
    except ValueError:
        tr = np.nan
    os_ = max(0.0, (f.max() - 1) * 100)
    out = np.where(np.abs(f - 1) > 0.02)[0]
    ts = (t[out[-1]] - t0) if len(out) else 0.0
    return tr, os_, ts
