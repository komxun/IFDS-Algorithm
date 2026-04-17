"""Weather / environmental constraint field loading and interpolation.

Ports ``initialize_constraint_matrix.m`` and ``Weather_map_Generator.m``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.io import loadmat


@dataclass
class WeatherField:
    """Static or dynamic weather constraint field.

    ``mat[i, j, t]`` holds the weather value at MATLAB index ``(i, j, t)``.
    MATLAB uses 1-based ``weatherMat(xx+1, yy+101)`` which translates to
    ``mat[int(round(xx)), int(round(yy)) + 100]`` in 0-based Python.

    Each time slice exposes a value interpolator and two gradient interpolators
    (``dwdx``, ``dwdy``), matching MATLAB's ``WMCell``, ``dwdxCell``, ``dwdyCell``.
    """

    mat: np.ndarray
    mat_mod: np.ndarray
    b_l: float
    b_u: float
    value_interp: list[RegularGridInterpolator]
    dwdx_interp: list[RegularGridInterpolator]
    dwdy_interp: list[RegularGridInterpolator]

    @property
    def n_frames(self) -> int:
        return self.mat.shape[2]

    def sample(self, t: int, x: float, y: float) -> tuple[float, float, float]:
        """Return ``(omega, dwdx, dwdy)`` at MATLAB coords ``(x+1, y+101)``."""
        xi = float(np.clip(x, 0, self.mat.shape[0] - 1))
        yi = float(np.clip(y + 100, 0, self.mat.shape[1] - 1))
        # Interpolators are built over MATLAB-style axes; query as (x, y) in MATLAB idx.
        pt = np.array([[xi + 1, yi + 1]])
        return (
            float(self.value_interp[t](pt)),
            float(self.dwdx_interp[t](pt)),
            float(self.dwdy_interp[t](pt)),
        )


def _build_interpolators(mat: np.ndarray) -> tuple[
    list[RegularGridInterpolator], list[RegularGridInterpolator], list[RegularGridInterpolator]
]:
    """Build value/dwdx/dwdy interpolators for each time slice.

    Matches the MATLAB loop in ``initialize_constraint_matrix.m``:
    ``WMCell{j} = griddedInterpolant(weatherMat(:,:,j)')`` (transposed), with
    gradient computed on the (x,y) meshgrid and similarly transposed.
    """
    nx, ny, nt = mat.shape
    xspace = np.arange(1, nx + 1, dtype=float)
    yspace = np.arange(1, ny + 1, dtype=float)

    val_list: list[RegularGridInterpolator] = []
    dx_list: list[RegularGridInterpolator] = []
    dy_list: list[RegularGridInterpolator] = []
    for j in range(nt):
        z = mat[:, :, j].T  # griddedInterpolant(M') in MATLAB
        val_list.append(
            RegularGridInterpolator((xspace, yspace), z.T, bounds_error=False, fill_value=None)
        )
        grad_x, grad_y = np.gradient(z, xspace, yspace)
        dx_list.append(
            RegularGridInterpolator((xspace, yspace), grad_x.T, bounds_error=False, fill_value=None)
        )
        dy_list.append(
            RegularGridInterpolator((xspace, yspace), grad_y.T, bounds_error=False, fill_value=None)
        )
    return val_list, dx_list, dy_list


def load_weather_field(
    path: str | Path,
    b_l: float,
    b_u: float,
) -> WeatherField:
    """Load a ``WeatherMat_*.mat`` file and build interpolators.

    Parameters mirror ``initialize_constraint_matrix.m`` where
    ``weatherMatMod`` clips to ``[b_l, 1]``.
    """
    path = Path(path)
    mat_dict = loadmat(str(path))
    mat = np.asarray(mat_dict["weatherMat"], dtype=float)
    mat_mod = mat.copy()
    mat_mod[mat_mod < b_l] = b_l
    mat_mod[mat_mod > b_u] = 1.0
    val_i, dx_i, dy_i = _build_interpolators(mat)
    return WeatherField(
        mat=mat, mat_mod=mat_mod, b_l=b_l, b_u=b_u,
        value_interp=val_i, dwdx_interp=dx_i, dwdy_interp=dy_i,
    )


def generate_weather_map(
    nx: int = 200,
    ny: int = 200,
    tmax: int = 30,
    num_seed: int = 69,
    f: float = 0.015,
    scale: float = 1.22,
) -> np.ndarray:
    """Generate a synthetic weather / constraint matrix via low-pass FFT noise.

    Port of ``Weather_map_Generator.m``. Returns ``(nx, ny, tmax)`` array in [0, 1].
    """
    rng = np.random.default_rng(num_seed)
    mat = np.zeros((nx, ny, tmax), dtype=float)
    base = rng.random((nx, ny))

    nx1 = 1 + int(0.999 * f * nx)
    nx2 = nx - nx1 + 1
    ny1 = 1 + int(0.999 * f * ny)
    ny2 = ny - nx1 + 1

    for t in range(1, tmax + 1):
        F = np.fft.fft2(base)
        F[nx1 - 1 : nx2 - int(0.5 * np.sin(t / 2)), :] = 0
        F[:, ny1 - 1 : ny2 - int(0.5 * np.cos(t / 2))] = 0
        f_ifft = np.real(np.fft.ifft2(F))
        lo, hi = f_ifft.min(), f_ifft.max()
        f_ifft = (f_ifft - lo) / (hi - lo)
        f_ifft *= scale
        f_ifft[f_ifft > 1] = 1
        mat[:, :, t - 1] = f_ifft
    return mat
