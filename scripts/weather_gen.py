"""Generate & save a synthetic weather constraint matrix.

Port of ``Weather_map_Generator.m``. Writes both ``WeatherMat_<seed>.mat`` (for
MATLAB compatibility) and ``WeatherMat_<seed>.npz`` into ``data/``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import savemat

from ifds.weather import generate_weather_map

REPO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = REPO_ROOT / "data"


def run(num_seed: int = 69, nx: int = 200, ny: int = 200, tmax: int = 30,
        f: float = 0.015, save: bool = False, *, plot: bool = True) -> np.ndarray:
    mat = generate_weather_map(nx=nx, ny=ny, tmax=tmax, num_seed=num_seed, f=f)
    if save:
        DATA_DIR.mkdir(exist_ok=True)
        mat_path = DATA_DIR / f"WeatherMat_{num_seed}.mat"
        npz_path = DATA_DIR / f"WeatherMat_{num_seed}.npz"
        savemat(str(mat_path), {"weatherMat": mat})
        np.savez_compressed(str(npz_path), weatherMat=mat)
        print(f"Saved {mat_path} and {npz_path}")
    if plot:
        fig, ax = plt.subplots()
        im = ax.imshow(mat[:, :, 0], cmap="turbo", origin="lower", aspect="equal")
        fig.colorbar(im)
        ax.set_title(f"numSeed={num_seed}, f={f}, t=0")
        plt.show()
    return mat


def main() -> None:
    p = argparse.ArgumentParser(description="Generate weather constraint map")
    p.add_argument("--seed", type=int, default=69)
    p.add_argument("--nx", type=int, default=200)
    p.add_argument("--ny", type=int, default=200)
    p.add_argument("--tmax", type=int, default=30)
    p.add_argument("--f", type=float, default=0.015)
    p.add_argument("--save", action="store_true")
    p.add_argument("--no-plot", dest="plot", action="store_false")
    args = p.parse_args()
    run(num_seed=args.seed, nx=args.nx, ny=args.ny, tmax=args.tmax,
        f=args.f, save=args.save, plot=args.plot)


if __name__ == "__main__":
    main()
