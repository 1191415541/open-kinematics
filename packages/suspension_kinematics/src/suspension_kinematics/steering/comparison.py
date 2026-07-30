"""Compare projected 2D and direct 3D two-segment steering results."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Sequence

from suspension_kinematics.steering import (
    TwoSegmentSteeringHardpoints3D,
    compare_two_segment_2d_and_3d,
    load_two_segment_steering_hardpoints_csv,
)

DEFAULT_OUTPUT_DIR = Path("scripts/plots")
DEFAULT_OUTPUT_CSV = DEFAULT_OUTPUT_DIR / "two_segment_2d_3d_comparison.csv"
DEFAULT_OUTPUT_PNG = DEFAULT_OUTPUT_DIR / "two_segment_2d_3d_comparison.png"
DEFAULT_PITMAN_START_DEG = -20.0
DEFAULT_PITMAN_STOP_DEG = 20.0
DEFAULT_PITMAN_STEP_DEG = 2.0


def pitman_angle_sweep_deg(
    start_deg: float = DEFAULT_PITMAN_START_DEG,
    stop_deg: float = DEFAULT_PITMAN_STOP_DEG,
    step_deg: float = DEFAULT_PITMAN_STEP_DEG,
) -> list[float]:
    """Build an inclusive pitman-angle sweep."""
    if step_deg <= 0.0:
        raise ValueError("step_deg must be positive")
    count = int(round((stop_deg - start_deg) / step_deg))
    if count < 0:
        raise ValueError("stop_deg must be greater than or equal to start_deg")
    return [float(start_deg + index * step_deg) for index in range(count + 1)]


def build_comparison_rows(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angles_deg: Sequence[float],
) -> list[dict[str, float]]:
    """Return flat comparison rows for one hardpoint set."""
    rows: list[dict[str, float]] = []
    guess = (0.0, 0.0)
    for pitman_angle_deg in pitman_angles_deg:
        comparison = compare_two_segment_2d_and_3d(
            hardpoints,
            pitman_angle_deg=float(pitman_angle_deg),
            initial_guess_deg=guess,
        )
        rows.append(
            {
                "pitman_angle_deg": float(pitman_angle_deg),
                "left_wheel_angle_2d_deg": comparison.solve_2d.left_wheel_angle_deg,
                "right_wheel_angle_2d_deg": comparison.solve_2d.right_wheel_angle_deg,
                "left_wheel_angle_3d_deg": comparison.solve_3d.left_wheel_angle_deg,
                "right_wheel_angle_3d_deg": comparison.solve_3d.right_wheel_angle_deg,
                "left_wheel_angle_delta_deg": comparison.left_wheel_angle_delta_deg,
                "right_wheel_angle_delta_deg": comparison.right_wheel_angle_delta_deg,
                "max_abs_wheel_angle_delta_deg": (
                    comparison.max_abs_wheel_angle_delta_deg
                ),
                "left_tie_rod_residual_2d": comparison.solve_2d.left_tie_rod_residual,
                "right_tie_rod_residual_2d": comparison.solve_2d.right_tie_rod_residual,
                "left_tie_rod_residual_3d": comparison.solve_3d.left_tie_rod_residual,
                "right_tie_rod_residual_3d": comparison.solve_3d.right_tie_rod_residual,
                "left_wheel_center_dx": float(comparison.left_wheel_center_delta_2d[0]),
                "left_wheel_center_dy": float(comparison.left_wheel_center_delta_2d[1]),
                "right_wheel_center_dx": float(
                    comparison.right_wheel_center_delta_2d[0]
                ),
                "right_wheel_center_dy": float(
                    comparison.right_wheel_center_delta_2d[1]
                ),
                "left_tie_rod_pickup_dx": float(
                    comparison.left_tie_rod_pickup_delta_2d[0]
                ),
                "left_tie_rod_pickup_dy": float(
                    comparison.left_tie_rod_pickup_delta_2d[1]
                ),
                "right_tie_rod_pickup_dx": float(
                    comparison.right_tie_rod_pickup_delta_2d[0]
                ),
                "right_tie_rod_pickup_dy": float(
                    comparison.right_tie_rod_pickup_delta_2d[1]
                ),
            }
        )
        guess = (
            comparison.solve_3d.left_wheel_angle_deg,
            comparison.solve_3d.right_wheel_angle_deg,
        )
    return rows


def write_comparison_csv(rows: Sequence[dict[str, float]], output_path: Path) -> None:
    """Write comparison rows to CSV."""
    if not rows:
        raise ValueError("rows must not be empty")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_comparison_rows(
    rows: Sequence[dict[str, float]],
    output_path: Path,
) -> Path | None:
    """Save a quick comparison chart when matplotlib is available."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    pitman = [row["pitman_angle_deg"] for row in rows]
    left_2d = [row["left_wheel_angle_2d_deg"] for row in rows]
    left_3d = [row["left_wheel_angle_3d_deg"] for row in rows]
    right_2d = [row["right_wheel_angle_2d_deg"] for row in rows]
    right_3d = [row["right_wheel_angle_3d_deg"] for row in rows]
    delta = [row["max_abs_wheel_angle_delta_deg"] for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(pitman, left_2d, "o-", label="left 2D")
    axes[0].plot(pitman, left_3d, "o--", label="left 3D")
    axes[0].plot(pitman, right_2d, "s-", label="right 2D")
    axes[0].plot(pitman, right_3d, "s--", label="right 3D")
    axes[0].set_ylabel("Wheel angle [deg]")
    axes[0].set_title("Two-segment steering: 2D projection vs 3D solve")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(pitman, delta, "d-", color="#d62728", label="max abs angle delta")
    axes[1].set_xlabel("Pitman angle [deg]")
    axes[1].set_ylabel("|3D - 2D| [deg]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def run_comparison(
    csv_path: Path,
    output_csv: Path = DEFAULT_OUTPUT_CSV,
    output_png: Path | None = DEFAULT_OUTPUT_PNG,
    *,
    start_deg: float = DEFAULT_PITMAN_START_DEG,
    stop_deg: float = DEFAULT_PITMAN_STOP_DEG,
    step_deg: float = DEFAULT_PITMAN_STEP_DEG,
) -> dict[str, Any]:
    """Run the full CSV-driven 2D/3D comparison workflow."""
    hardpoints = load_two_segment_steering_hardpoints_csv(csv_path)
    angles = pitman_angle_sweep_deg(
        start_deg=start_deg,
        stop_deg=stop_deg,
        step_deg=step_deg,
    )
    rows = build_comparison_rows(hardpoints, angles)
    write_comparison_csv(rows, output_csv)
    plotted = None if output_png is None else plot_comparison_rows(rows, output_png)
    max_delta = max(row["max_abs_wheel_angle_delta_deg"] for row in rows)
    return {
        "rows": rows,
        "output_csv": output_csv,
        "output_png": plotted,
        "max_abs_wheel_angle_delta_deg": max_delta,
    }


def _parse_args() -> Any:
    import argparse

    parser = argparse.ArgumentParser(
        description="Compare 2D-projected and direct 3D two-segment steering results.",
    )
    parser.add_argument("csv_path", type=Path, help="Input steering hardpoint CSV")
    parser.add_argument(
        "--out-csv",
        type=Path,
        default=DEFAULT_OUTPUT_CSV,
        help="Output CSV path",
    )
    parser.add_argument(
        "--out-png",
        type=Path,
        default=DEFAULT_OUTPUT_PNG,
        help="Optional output plot path",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plot generation",
    )
    parser.add_argument("--start-deg", type=float, default=DEFAULT_PITMAN_START_DEG)
    parser.add_argument("--stop-deg", type=float, default=DEFAULT_PITMAN_STOP_DEG)
    parser.add_argument("--step-deg", type=float, default=DEFAULT_PITMAN_STEP_DEG)
    return parser.parse_args()


def main() -> None:
    """Run the comparison script from the command line."""
    args = _parse_args()
    result = run_comparison(
        args.csv_path,
        output_csv=args.out_csv,
        output_png=None if args.no_plot else args.out_png,
        start_deg=args.start_deg,
        stop_deg=args.stop_deg,
        step_deg=args.step_deg,
    )
    print(f"wrote {result['output_csv']}")
    if isinstance(result["output_png"], Path):
        print(f"wrote {result['output_png']}")
    print(
        f"max_abs_wheel_angle_delta_deg={result['max_abs_wheel_angle_delta_deg']:.6f}"
    )


if __name__ == "__main__":
    main()
