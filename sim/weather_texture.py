"""Weather field → PyBullet ground-plane texture.

Renders a ``WeatherField`` time-slice as an RGB image using *matplotlib*,
draws the ``b_u`` contour as a white line, and applies it as a PyBullet
texture on the custom ground plane created by :class:`Aviary`.

The ground plane is scaled to cover the IFDS coordinate domain:
    x ∈ [0, 200],  y ∈ [-100, 100],  z = 0.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

# Use non-interactive backend so texture generation never opens a window.
matplotlib.use("Agg")


def weather_to_rgba(
    weather,
    frame: int = 0,
    cmap_name: str = "turbo",
    alpha: float = 0.85,
) -> np.ndarray:
    """Convert a weather time-slice to an RGBA uint8 image.

    Parameters
    ----------
    weather : WeatherField
        Must have ``.mat`` (shape ``(nx, ny, nt)``) and ``.b_u``.
    frame : int
        Time index.
    cmap_name : str
        Matplotlib colour-map name.
    alpha : float
        Global alpha for the heatmap.

    Returns
    -------
    np.ndarray
        ``(ny, nx, 4)`` uint8 RGBA image suitable for ``PIL.Image.save()``.
    """
    mat = weather.mat
    data = mat[:, :, frame].T  # shape (ny, nx) — rows = y, cols = x

    cmap = matplotlib.colormaps.get_cmap(cmap_name)
    vmin, vmax = float(np.min(mat)), float(np.max(mat))
    if np.isclose(vmin, vmax):
        vmax = vmin + 1.0
    normed = (data - vmin) / (vmax - vmin)
    rgba_float = cmap(normed)  # (ny, nx, 4)
    rgba_float[..., 3] = alpha

    # Draw b_u contour as white pixels (use matplotlib contour on the data)
    fig, ax = plt.subplots(1, 1, figsize=(data.shape[1] / 100, data.shape[0] / 100), dpi=100)
    ax.set_position([0, 0, 1, 1])
    ax.set_xlim(0, data.shape[1])
    ax.set_ylim(0, data.shape[0])
    ax.axis("off")
    ax.contour(data, levels=[weather.b_u], colors="white", linewidths=2)
    fig.canvas.draw()

    # Rasterise the contour overlay
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    w, h = fig.canvas.get_width_height()
    contour_rgba = buf.reshape(h, w, 4)
    plt.close(fig)

    # Resize contour overlay to match data shape if needed
    from PIL import Image as _PILImage

    contour_img = _PILImage.fromarray(contour_rgba, "RGBA").resize(
        (data.shape[1], data.shape[0]), _PILImage.LANCZOS
    )
    contour_arr = np.array(contour_img)

    # Composite: where contour is nearly white, replace pixels
    white_mask = (contour_arr[:, :, 0] > 200) & (contour_arr[:, :, 1] > 200) & (contour_arr[:, :, 2] > 200)
    rgba_float[white_mask] = [1.0, 1.0, 1.0, 1.0]

    return (rgba_float * 255).astype(np.uint8)


def weather_to_rgb(
    weather,
    frame: int = 0,
    cmap_name: str = "turbo",
    resolution: tuple[int, int] = (512, 512),
) -> np.ndarray:
    """Render a weather time-slice as a complete matplotlib figure.

    Uses ``plt.savefig`` to get a clean, correctly-oriented image with the
    ``b_u`` contour baked in.  Returns an ``(H, W, 3)`` uint8 RGB array.

    Parameters
    ----------
    weather : WeatherField
    frame : int
    cmap_name : str
    resolution : tuple[int, int]
        ``(width, height)`` in pixels.

    Returns
    -------
    np.ndarray
        ``(H, W, 3)`` uint8 RGB image.
    """
    from PIL import Image as _PILImage

    mat = weather.mat
    data = mat[:, :, frame].T  # (ny, nx) — rows=y, cols=x

    dpi = 100
    w_in = resolution[0] / dpi
    h_in = resolution[1] / dpi

    fig, ax = plt.subplots(1, 1, figsize=(w_in, h_in), dpi=dpi)
    ax.set_position([0, 0, 1, 1])
    ax.imshow(
        data,
        origin="lower",
        aspect="auto",
        cmap=cmap_name,
        vmin=float(np.min(mat)),
        vmax=float(np.max(mat)),
    )
    ax.contour(data, levels=[weather.b_u], colors="white", linewidths=1.5)
    ax.axis("off")
    fig.canvas.draw()

    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    cw, ch = fig.canvas.get_width_height()
    img_arr = buf.reshape(ch, cw, 4)[:, :, :3].copy()  # drop alpha → RGB
    plt.close(fig)
    return img_arr


def _save_rgb_png(rgb: np.ndarray, path: str | Path) -> str:
    """Save an RGB uint8 array to PNG."""
    from PIL import Image as _PILImage

    img = _PILImage.fromarray(rgb, "RGB")
    img.save(str(path))
    return str(path)


class WeatherGroundPlane:
    """Manages a custom ground plane with a weather-field texture in PyBullet.

    The Aviary creates a large box at z≈0 covering the IFDS domain.  This
    class renders the weather field to a PNG and applies it as a PyBullet
    texture on that body.

    Parameters
    ----------
    client_id : int
        PyBullet physics-client id.
    plane_id : int
        Body id of the ground plane already loaded in the aviary.
    domain_x : tuple[float, float]
        ``(x_min, x_max)`` of the weather field (IFDS coords).
    domain_y : tuple[float, float]
        ``(y_min, y_max)`` of the weather field (IFDS coords).
    """

    def __init__(
        self,
        client_id: int,
        plane_id: int,
        domain_x: tuple[float, float] = (0.0, 200.0),
        domain_y: tuple[float, float] = (-100.0, 100.0),
    ):
        self._client = client_id
        self._plane_id = plane_id
        self._domain_x = domain_x
        self._domain_y = domain_y
        self._tmp_dir = tempfile.mkdtemp(prefix="ifds_weather_")
        self._texture_id: int | None = None
        self._last_frame: int | None = None

    def update(self, weather, frame: int = 0, *, force: bool = False) -> None:
        """Render the weather field for *frame* and apply as ground texture.

        Skips rendering if *frame* has not changed since the last call
        (unless ``force=True``).
        """
        if not force and frame == self._last_frame:
            return
        self._last_frame = frame

        rgb = weather_to_rgb(weather, frame=frame, resolution=(1024, 1024))
        png_path = Path(self._tmp_dir) / f"weather_{frame:04d}.png"
        _save_rgb_png(rgb, png_path)

        tex = p.loadTexture(str(png_path), physicsClientId=self._client)
        p.changeVisualShape(
            self._plane_id, -1,
            textureUniqueId=tex,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.1, 0.1, 0.1],
            physicsClientId=self._client,
        )
        self._texture_id = tex
