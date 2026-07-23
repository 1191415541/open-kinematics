"""Matplotlib drawing helpers for the suspension GUI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import cast

import numpy as np
from mpl_toolkits.mplot3d.axes3d import Axes3D

from kinematics.gui.suspension.workbench import suspension_internal_to_gui_vec3
from kinematics.state import SuspensionState
from kinematics.suspensions.base import Suspension
from kinematics.visualization.main import LinkVisualization
from kinematics.visualization.main import SuspensionVisualizer, WheelVisualization
from kinematics.visualization.plots import (
    compute_bounds_from_positions,
    configure_3d_axis,
    plot_suspension_on_axis,
)


PREVIEW_VIEW_PRESETS: dict[str, tuple[float, float]] = {
    "iso": (20.0, 45.0),
    "xy": (90.0, -90.0),  # top: X rearward, Y rightward
    "xz": (0.0, -90.0),  # side: X rearward, Z up
    "yz": (0.0, 0.0),  # rear/front: Y rightward, Z up
    "zy": (0.0, 180.0),  # opposite side of yz
}


@dataclass(frozen=True)
class _PreviewRenderSignature:
    """Signature used to detect when preview artists must be rebuilt."""

    links: tuple[
        tuple[
            tuple[int, ...],
            str,
            str,
            float,
            str,
            str,
            float,
        ],
        ...,
    ]
    wheel_diameter: float
    wheel_width: float
    wheel_points: int
    wheel_bands: int


class SuspensionPreviewRenderer:
    """Reusable renderer that updates existing 3D artists for smoother previews."""

    PREVIEW_WHEEL_POINTS = 24
    PREVIEW_WHEEL_BANDS = 12
    FULL_WHEEL_POINTS = 40
    FULL_WHEEL_BANDS = 20

    def __init__(self) -> None:
        self._signature: _PreviewRenderSignature | None = None
        self._visualizer: SuspensionVisualizer | None = None
        self._link_artists: list[object] | None = None
        self._wheel_artists: dict[str, list[object]] | None = None
        self._wheel_bands = self.FULL_WHEEL_BANDS

    def reset(self) -> None:
        """Drop cached artists/signature so the next draw performs a full rebuild."""
        self._signature = None
        self._visualizer = None
        self._link_artists = None
        self._wheel_artists = None
        self._wheel_bands = self.FULL_WHEEL_BANDS

    def draw(
        self,
        ax: Axes3D,
        suspension: Suspension,
        state: SuspensionState,
        *,
        preserve_view: bool,
        preview_mode: bool,
    ) -> None:
        """Render one state, reusing artists whenever possible."""
        if suspension.config is None:
            raise ValueError("Suspension has no configuration")

        gui_state = _state_to_gui_coordinates(state)
        wheel_cfg = suspension.config.wheel
        # Once a scene exists, keep its wheel tessellation so preview/full
        # redraws update geometry without rebuilding artists or resetting camera.
        if self._signature is not None and self._visualizer is not None:
            wheel_points = self._signature.wheel_points
            wheel_bands = self._signature.wheel_bands
        else:
            wheel_points = (
                self.PREVIEW_WHEEL_POINTS if preview_mode else self.FULL_WHEEL_POINTS
            )
            wheel_bands = (
                self.PREVIEW_WHEEL_BANDS if preview_mode else self.FULL_WHEEL_BANDS
            )
        links = suspension.get_visualization_links()
        signature = _build_signature(
            links=links,
            wheel_diameter=wheel_cfg.tire.nominal_radius * 2,
            wheel_width=wheel_cfg.tire.section_width,
            wheel_points=wheel_points,
            wheel_bands=wheel_bands,
        )

        should_rebuild = (
            self._signature != signature
            or self._visualizer is None
            or self._link_artists is None
            or self._wheel_artists is None
        )

        if should_rebuild:
            self._rebuild_scene(
                ax=ax,
                suspension=suspension,
                state=state,
                preserve_view=preserve_view,
                links=links,
                wheel_diameter=wheel_cfg.tire.nominal_radius * 2,
                wheel_width=wheel_cfg.tire.section_width,
                wheel_points=wheel_points,
                wheel_bands=wheel_bands,
                signature=signature,
            )
            return

        self._visualizer.update_links(self._link_artists, gui_state.positions)
        self._visualizer.update_wheel(
            self._wheel_artists,
            gui_state.positions,
            num_bands=self._wheel_bands,
        )

    def _rebuild_scene(
        self,
        *,
        ax: Axes3D,
        suspension: Suspension,
        state: SuspensionState,
        preserve_view: bool,
        links: list[LinkVisualization],
        wheel_diameter: float,
        wheel_width: float,
        wheel_points: int,
        wheel_bands: int,
        signature: _PreviewRenderSignature,
    ) -> None:
        gui_state = _state_to_gui_coordinates(state)
        limits = (ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d())
        view = (float(ax.elev), float(ax.azim))
        ax.clear()
        if preserve_view:
            ax.set_xlim3d(limits[0])
            ax.set_ylim3d(limits[1])
            ax.set_zlim3d(limits[2])
            ax.view_init(elev=view[0], azim=view[1])
            ax.set_proj_type("ortho")
            ax.set_box_aspect([1, 1, 1])  # type: ignore[arg-type]
            _set_preview_axis_labels(ax)
        else:
            _, _, (x_mid, y_mid, z_mid, max_range) = compute_bounds_from_positions(
                gui_state.positions
            )
            configure_3d_axis(cast(Axes3D, ax), "iso", x_mid, y_mid, z_mid, max_range)
            _set_preview_axis_labels(ax)

        self._visualizer = SuspensionVisualizer(
            links,
            WheelVisualization(
                diameter=wheel_diameter,
                width=wheel_width,
                num_points=wheel_points,
            ),
        )
        self._link_artists = self._visualizer.draw_links(ax, gui_state.positions)
        self._wheel_artists = self._visualizer.draw_wheel(
            ax,
            gui_state.positions,
            num_bands=wheel_bands,
        )
        _apply_preview_legend_layout(ax)
        self._signature = signature
        self._wheel_bands = wheel_bands


def apply_preview_view_plane(
    ax: Axes3D,
    plane: str,
    *,
    positions: dict | None = None,
    fit_bounds: bool = True,
) -> None:
    """
    Align the preview camera to a named plane.

    Supported planes: iso, xy, xz, yz, zy.
    """
    key = plane.strip().lower()
    if key not in PREVIEW_VIEW_PRESETS:
        raise ValueError(f"Unsupported preview plane '{plane}'")
    elev, azim = PREVIEW_VIEW_PRESETS[key]
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("ortho")
    ax.set_box_aspect([1, 1, 1])  # type: ignore[arg-type]
    _set_preview_axis_labels(ax)
    if fit_bounds and positions:
        _, _, (x_mid, y_mid, z_mid, max_range) = compute_bounds_from_positions(
            positions
        )
        ax.set_xlim3d([x_mid - max_range / 2, x_mid + max_range / 2])
        ax.set_ylim3d([y_mid - max_range / 2, y_mid + max_range / 2])
        ax.set_zlim3d([z_mid - max_range / 2, z_mid + max_range / 2])


def _build_signature(
    *,
    links: list[LinkVisualization],
    wheel_diameter: float,
    wheel_width: float,
    wheel_points: int,
    wheel_bands: int,
) -> _PreviewRenderSignature:
    return _PreviewRenderSignature(
        links=tuple(
            (
                tuple(int(point_id) for point_id in link.points),
                str(link.color),
                str(link.label),
                float(link.linewidth),
                str(link.linestyle),
                str(link.marker),
                float(link.markersize),
            )
            for link in links
        ),
        wheel_diameter=float(wheel_diameter),
        wheel_width=float(wheel_width),
        wheel_points=int(wheel_points),
        wheel_bands=int(wheel_bands),
    )


def draw_suspension_preview(
    ax: Axes3D,
    suspension: Suspension,
    state: SuspensionState,
    *,
    preserve_view: bool = False,
    renderer: SuspensionPreviewRenderer | None = None,
    preview_mode: bool = False,
) -> None:
    """Draw one suspension state on a 3D axis."""
    if renderer is not None:
        renderer.draw(
            ax,
            suspension,
            state,
            preserve_view=preserve_view,
            preview_mode=preview_mode,
        )
        return

    if suspension.config is None:
        raise ValueError("Suspension has no configuration")

    gui_state = _state_to_gui_coordinates(state)
    wheel_cfg = suspension.config.wheel
    visualizer = SuspensionVisualizer(
        suspension.get_visualization_links(),
        WheelVisualization(
            diameter=wheel_cfg.tire.nominal_radius * 2,
            width=wheel_cfg.tire.section_width,
        ),
    )
    limits = (ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d())
    view = (float(ax.elev), float(ax.azim))
    ax.clear()
    if preserve_view:
        ax.set_xlim3d(limits[0])
        ax.set_ylim3d(limits[1])
        ax.set_zlim3d(limits[2])
        ax.view_init(elev=view[0], azim=view[1])
        ax.set_proj_type("ortho")
        ax.set_box_aspect([1, 1, 1])  # type: ignore[arg-type]
        _set_preview_axis_labels(ax)
    else:
        _, _, (x_mid, y_mid, z_mid, max_range) = compute_bounds_from_positions(
            gui_state.positions
        )
        configure_3d_axis(cast(Axes3D, ax), "iso", x_mid, y_mid, z_mid, max_range)
        _set_preview_axis_labels(ax)
    plot_suspension_on_axis(cast(Axes3D, ax), visualizer, gui_state.positions, "iso")
    _apply_preview_legend_layout(ax)


def draw_suspension_curve(
    ax,
    rows: list[dict[str, float | bool | None]],
    x_key: str,
    y_key: str,
) -> None:
    """Draw a suspension output curve."""
    ax.clear()
    x_values = [float(row[x_key]) for row in rows if _is_number(row.get(x_key))]
    y_values = [float(row[y_key]) for row in rows if _is_number(row.get(y_key))]
    if x_values and len(x_values) == len(y_values):
        ax.plot(x_values, y_values, marker="o")
    ax.set_title("Suspension K Curves")
    ax.set_xlabel(x_key)
    ax.set_ylabel(y_key)
    ax.grid(True, alpha=0.25)
    ax.figure.subplots_adjust(left=0.18, right=0.98, bottom=0.22, top=0.88)


def draw_suspension_curve_plot(
    ax,
    rows: list[dict[str, float | bool | None]],
    curves: list[tuple[str, str, str]],
) -> None:
    """Draw managed suspension output curves."""
    ax.clear()
    for x_key, y_key, label in curves:
        pairs = [
            (float(row[x_key]), float(row[y_key]))
            for row in rows
            if _is_number(row.get(x_key)) and _is_number(row.get(y_key))
        ]
        if not pairs:
            continue
        x_values, y_values = zip(*pairs)
        curve_label = label or f"{y_key} vs {x_key}"
        ax.plot(x_values, y_values, marker="o", label=curve_label)
    ax.set_title("Suspension K Curves")
    ax.set_xlabel(curves[0][0] if curves else "x")
    ax.set_ylabel("selected output")
    ax.grid(True, alpha=0.25)
    if curves:
        ax.legend(loc="best")
    ax.figure.subplots_adjust(left=0.18, right=0.98, bottom=0.22, top=0.88)


def _is_number(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool)


def _state_to_gui_coordinates(state: SuspensionState) -> SuspensionState:
    return SuspensionState(
        positions={
            point_id: suspension_internal_to_gui_vec3(position)
            for point_id, position in state.positions.items()
        },
        free_points=set(state.free_points),
    )


def _set_preview_axis_labels(ax: Axes3D) -> None:
    ax.set_xlabel("X rearward [mm]")
    ax.set_ylabel("Y rightward [mm]")
    ax.set_zlabel("Z upward [mm]")


def _apply_preview_legend_layout(ax: Axes3D) -> None:
    """Keep the preview legend in the page corner instead of over the suspension."""
    ax.set_position((0.28, 0.06, 0.7, 0.88))
    ax.legend(
        loc="upper left",
        bbox_to_anchor=(0.02, 0.96),
        bbox_transform=ax.figure.transFigure,
        borderaxespad=0.0,
    )
