"""Generate a standalone Adams/native comparison plot for one real run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

GROUPS: tuple[tuple[str, tuple[str, ...]], ...] = (
    (
        "Body state",
        (
            "sprung_body.heave",
            "sprung_body.pitch",
            "sprung_body.roll",
            "sprung_body.heave_velocity",
            "sprung_body.pitch_rate",
            "sprung_body.roll_rate",
            "sprung_body.heave_acceleration",
        ),
    ),
    (
        "Wheel and suspension",
        (
            "left.wheel_center_z",
            "right.wheel_center_z",
            "left.wheel_center_z_velocity",
            "right.wheel_center_z_velocity",
            "left.wheel_center_z_acceleration",
            "right.wheel_center_z_acceleration",
            "left.suspension_deflection",
            "right.suspension_deflection",
        ),
    ),
    (
        "Tire forces",
        (
            "left.tire_normal_force",
            "right.tire_normal_force",
            "left.tire_longitudinal_force",
            "right.tire_longitudinal_force",
            "left.tire_lateral_force",
            "right.tire_lateral_force",
        ),
    ),
    (
        "Spring and damper",
        (
            "left.spring_force",
            "right.spring_force",
            "left.damper_force",
            "right.damper_force",
        ),
    ),
    (
        "Wheel spin",
        (
            "left.wheel_spin",
            "right.wheel_spin",
        ),
    ),
    (
        "Fixture wrench",
        (
            "fixture.force_x",
            "fixture.force_y",
            "fixture.force_z",
            "fixture.moment_x",
            "fixture.moment_y",
            "fixture.moment_z",
        ),
    ),
)

_ROOT_ID = "road-sine-adams-native-comparison"


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected an object in {path}")
    return value


def _load_payload(artifact_root: Path) -> dict[str, Any]:
    adams = _read_json(artifact_root / "adams_refined_history.json")
    native = _read_json(artifact_root / "native" / "native_refined_history.json")
    comparison = _read_json(artifact_root / "dynamic_comparison.json")
    adams_time = adams.get("time")
    native_time = native.get("time")
    if not isinstance(adams_time, list) or not isinstance(native_time, list):
        raise ValueError("history files must contain time arrays")
    if len(adams_time) != len(native_time) or any(
        not math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-12)
        for a, b in zip(adams_time, native_time)
    ):
        raise ValueError("Adams and native histories do not share a common grid")
    adams_channels = adams.get("channels")
    native_channels = native.get("channels")
    units = adams.get("units")
    metrics = comparison.get("channels")
    if not isinstance(adams_channels, dict) or not isinstance(native_channels, dict):
        raise ValueError("history files must contain channel mappings")
    if not isinstance(units, dict) or not isinstance(metrics, dict):
        raise ValueError("history/comparison files have incomplete metadata")

    channels: dict[str, dict[str, Any]] = {}
    for name, values in adams_channels.items():
        if name not in native_channels:
            raise ValueError(f"native history is missing channel {name!r}")
        if not isinstance(values, list) or not isinstance(native_channels[name], list):
            raise ValueError(f"channel {name!r} must contain arrays")
        if len(values) != len(adams_time) or len(native_channels[name]) != len(adams_time):
            raise ValueError(f"channel {name!r} length does not match the time grid")
        channels[name] = {
            "unit": str(units.get(name, "")),
            "adams": [float(value) for value in values],
            "native": [float(value) for value in native_channels[name]],
            "metric": {
                "nrmse": float(metrics[name]["nrmse"]),
                "maximum_absolute_error": float(
                    metrics[name]["maximum_absolute_error"]
                ),
                "passed": bool(metrics[name]["passed"]),
            },
        }

    expected = [name for _, names in GROUPS for name in names]
    if tuple(channels) != tuple(expected):
        raise ValueError("history channel order does not match the frozen 33-channel groups")
    start_s = float(adams_time[0])
    end_s = float(adams_time[-1])
    step_s = (
        float(adams_time[1]) - float(adams_time[0])
        if len(adams_time) > 1
        else 0.0
    )
    return {
        "time": [float(value) for value in adams_time],
        "channels": channels,
        "groups": [[title, list(names)] for title, names in GROUPS],
        "meta": {
            "sample_count": len(adams_time),
            "start_s": start_s,
            "end_s": end_s,
            "step_s": step_s,
            "comparison_passed": bool(comparison.get("passed")),
            "harmonic_performed": bool(
                comparison.get("harmonic_gate", {}).get("performed", False)
            ),
        },
    }


def _fragment(payload: dict[str, Any]) -> str:
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return """<div id="road-sine-adams-native-comparison">
  <div class="comparison-meta">
    <p class="comparison-context">Real Adams/Solver versus native | common public grid | 33 channels</p>
    <div class="comparison-legend" aria-label="Series legend">
      <span class="legend-item"><span class="legend-line legend-adams" aria-hidden="true"></span>Adams</span>
      <span class="legend-item"><span class="legend-line legend-native" aria-hidden="true"></span>native</span>
      <span class="legend-note">Hover a plot for exact samples</span>
    </div>
  </div>
  <div id="comparison-groups"></div>
</div>
<style>
#road-sine-adams-native-comparison {
  color: var(--foreground);
  font-family: inherit;
  width: 100%;
}
#road-sine-adams-native-comparison .comparison-meta {
  margin-bottom: 1.25rem;
}
#road-sine-adams-native-comparison .comparison-context {
  color: var(--foreground);
  margin: 0 0 0.5rem;
}
#road-sine-adams-native-comparison .comparison-legend {
  align-items: center;
  color: var(--muted-foreground);
  display: flex;
  flex-wrap: wrap;
  gap: 0.8rem 1.1rem;
  font-size: var(--font-size-small);
}
#road-sine-adams-native-comparison .legend-item {
  align-items: center;
  display: inline-flex;
  gap: 0.35rem;
}
#road-sine-adams-native-comparison .legend-line {
  display: inline-block;
  height: 0;
  width: 1.8rem;
}
#road-sine-adams-native-comparison .legend-adams {
  border-top: 2px solid var(--viz-series-1);
}
#road-sine-adams-native-comparison .legend-native {
  border-top: 2px dashed var(--viz-series-2);
}
#road-sine-adams-native-comparison .legend-note {
  color: var(--muted-foreground);
}
#road-sine-adams-native-comparison .comparison-group {
  margin: 1.5rem 0 2rem;
}
#road-sine-adams-native-comparison .comparison-group h2 {
  color: var(--foreground);
  font-size: var(--font-size-h3);
  font-weight: 500;
  margin: 0 0 0.7rem;
}
#road-sine-adams-native-comparison .comparison-grid {
  display: grid;
  gap: 1rem 1.25rem;
  grid-template-columns: repeat(2, minmax(0, 1fr));
}
#road-sine-adams-native-comparison .comparison-plot {
  margin: 0;
  min-width: 0;
}
#road-sine-adams-native-comparison .plot-caption {
  align-items: baseline;
  display: flex;
  flex-wrap: wrap;
  gap: 0.25rem 0.55rem;
  justify-content: space-between;
  margin-bottom: 0.25rem;
}
#road-sine-adams-native-comparison .plot-name {
  color: var(--foreground);
  font-weight: 500;
}
#road-sine-adams-native-comparison .plot-unit,
#road-sine-adams-native-comparison .plot-metric {
  color: var(--muted-foreground);
  font-size: var(--font-size-small);
}
#road-sine-adams-native-comparison .plot-metric {
  font-variant-numeric: tabular-nums;
}
#road-sine-adams-native-comparison .plot-wrap {
  position: relative;
}
#road-sine-adams-native-comparison .plot-svg {
  display: block;
  height: auto;
  overflow: visible;
  width: 100%;
}
#road-sine-adams-native-comparison .plot-grid,
#road-sine-adams-native-comparison .plot-axis {
  stroke: var(--border);
  stroke-width: 1;
  vector-effect: non-scaling-stroke;
}
#road-sine-adams-native-comparison .plot-zero {
  stroke: var(--muted-foreground);
  stroke-dasharray: 2 3;
  stroke-width: 1;
  vector-effect: non-scaling-stroke;
}
#road-sine-adams-native-comparison .plot-adams,
#road-sine-adams-native-comparison .plot-native {
  fill: none;
  stroke-linejoin: round;
  stroke-linecap: round;
  stroke-width: 1.6;
  vector-effect: non-scaling-stroke;
}
#road-sine-adams-native-comparison .plot-adams {
  stroke: var(--viz-series-1);
}
#road-sine-adams-native-comparison .plot-native {
  stroke: var(--viz-series-2);
  stroke-dasharray: 5 3;
}
#road-sine-adams-native-comparison .plot-focus {
  stroke: var(--muted-foreground);
  stroke-dasharray: 3 3;
  stroke-width: 1;
  vector-effect: non-scaling-stroke;
}
#road-sine-adams-native-comparison .plot-text {
  fill: var(--muted-foreground);
  font-size: var(--font-size-small);
  font-variant-numeric: tabular-nums;
}
#road-sine-adams-native-comparison .plot-hit {
  cursor: crosshair;
  fill: transparent;
}
#road-sine-adams-native-comparison .plot-tooltip {
  background: var(--popover);
  border: 1px solid var(--border);
  color: var(--popover-foreground);
  font-size: var(--font-size-tooltip);
  line-height: var(--line-height-tooltip);
  max-width: min(20rem, calc(100% - 0.5rem));
  padding: 0.3rem 0.45rem;
  pointer-events: none;
  position: absolute;
  white-space: nowrap;
  z-index: 2;
}
#road-sine-adams-native-comparison .plot-tooltip[hidden] {
  display: none;
}
@media (max-width: 560px) {
  #road-sine-adams-native-comparison .comparison-grid {
    grid-template-columns: minmax(0, 1fr);
  }
}
</style>
<script>
(() => {
  const root = document.getElementById("road-sine-adams-native-comparison");
  const payload = __PAYLOAD__;
  const plotWidth = 380;
  const plotHeight = 190;
  const margin = { left: 54, right: 12, top: 10, bottom: 27 };
  const innerWidth = plotWidth - margin.left - margin.right;
  const innerHeight = plotHeight - margin.top - margin.bottom;
  const svgNs = "http://www.w3.org/2000/svg";

  const clamp = (value, low, high) => Math.min(Math.max(value, low), high);
  const formatValue = (value) => {
    const absolute = Math.abs(value);
    if (absolute !== 0 && (absolute < 1e-3 || absolute >= 1e4)) {
      return value.toExponential(3);
    }
    return value.toPrecision(5);
  };
  const formatMetric = (value) => value.toPrecision(4);
  const addSvg = (parent, tag, attributes) => {
    const element = document.createElementNS(svgNs, tag);
    Object.entries(attributes).forEach(([key, value]) => {
      element.setAttribute(key, String(value));
    });
    parent.appendChild(element);
    return element;
  };
  const pathFor = (values, low, high) => {
    const span = high - low;
    return values.map((value, index) => {
      const x = margin.left + (index / (values.length - 1)) * innerWidth;
      const y = margin.top + ((high - value) / span) * innerHeight;
      return `${index === 0 ? "M" : "L"}${x.toFixed(2)} ${y.toFixed(2)}`;
    }).join(" ");
  };
  const makePlot = (name) => {
    const series = payload.channels[name];
    const all = series.adams.concat(series.native);
    const dataLow = Math.min(...all);
    const dataHigh = Math.max(...all);
    const dataSpan = dataHigh - dataLow;
    const padding = Math.max(dataSpan * 0.08, Math.max(Math.abs(dataLow), Math.abs(dataHigh)) * 0.02, 1e-9);
    const low = dataLow - padding;
    const high = dataHigh + padding;
    const figure = document.createElement("figure");
    figure.className = "comparison-plot";
    const caption = document.createElement("figcaption");
    caption.className = "plot-caption";
    const nameLabel = document.createElement("span");
    nameLabel.className = "plot-name";
    nameLabel.textContent = name;
    const unitLabel = document.createElement("span");
    unitLabel.className = "plot-unit";
    unitLabel.textContent = series.unit;
    const metricLabel = document.createElement("span");
    metricLabel.className = "plot-metric";
    metricLabel.textContent = `NRMSE ${formatMetric(series.metric.nrmse)} | max abs ${formatValue(series.metric.maximum_absolute_error)} ${series.unit}`;
    caption.append(nameLabel, unitLabel, metricLabel);
    figure.appendChild(caption);

    const wrap = document.createElement("div");
    wrap.className = "plot-wrap";
    const svg = document.createElementNS(svgNs, "svg");
    svg.classList.add("plot-svg");
    svg.setAttribute("viewBox", `0 0 ${plotWidth} ${plotHeight}`);
    svg.setAttribute("role", "img");
    const titleId = `${name.replace(/[^a-z0-9]+/gi, "-")}-title`;
    const descId = `${name.replace(/[^a-z0-9]+/gi, "-")}-desc`;
    svg.setAttribute("aria-labelledby", `${titleId} ${descId}`);
    addSvg(svg, "title", { id: titleId }).textContent = `${name}: Adams versus native`;
    addSvg(svg, "desc", { id: descId }).textContent = `Common-grid comparison in ${series.unit}; NRMSE ${formatMetric(series.metric.nrmse)}.`;
    const xLeft = margin.left;
    const xRight = margin.left + innerWidth;
    const yTop = margin.top;
    const yBottom = margin.top + innerHeight;
    [0, 0.5, 1].forEach((fraction) => {
      const y = yBottom - fraction * innerHeight;
      addSvg(svg, "line", { class: fraction === 0.5 && low < 0 && high > 0 ? "plot-zero" : "plot-grid", x1: xLeft, x2: xRight, y1: y, y2: y });
      addSvg(svg, "text", { class: "plot-text", x: xLeft - 7, y: y + 4, "text-anchor": "end" }).textContent = formatValue(low + fraction * (high - low));
    });
    [0, 0.5, 1].forEach((fraction) => {
      const x = xLeft + fraction * innerWidth;
      addSvg(svg, "line", { class: "plot-grid", x1: x, x2: x, y1: yTop, y2: yBottom });
      addSvg(svg, "text", { class: "plot-text", x, y: yBottom + 19, "text-anchor": fraction === 0 ? "start" : fraction === 1 ? "end" : "middle" }).textContent = `${(payload.time[0] + fraction * (payload.time[payload.time.length - 1] - payload.time[0])).toFixed(2)} s`;
    });
    addSvg(svg, "line", { class: "plot-axis", x1: xLeft, x2: xLeft, y1: yTop, y2: yBottom });
    addSvg(svg, "line", { class: "plot-axis", x1: xLeft, x2: xRight, y1: yBottom, y2: yBottom });
    addSvg(svg, "path", { class: "plot-adams", d: pathFor(series.adams, low, high) });
    addSvg(svg, "path", { class: "plot-native", d: pathFor(series.native, low, high) });
    const focus = addSvg(svg, "line", { class: "plot-focus", x1: xLeft, x2: xLeft, y1: yTop, y2: yBottom });
    focus.style.display = "none";
    const hit = addSvg(svg, "rect", { class: "plot-hit", x: xLeft, y: yTop, width: innerWidth, height: innerHeight });
    const tooltip = document.createElement("div");
    tooltip.className = "plot-tooltip";
    tooltip.hidden = true;
    wrap.append(svg, tooltip);
    figure.appendChild(wrap);

    const showValue = (event) => {
      const bounds = svg.getBoundingClientRect();
      const fraction = clamp((event.clientX - bounds.left) / bounds.width, 0, 1);
      const index = Math.round(fraction * (payload.time.length - 1));
      const x = xLeft + (index / (payload.time.length - 1)) * innerWidth;
      const cssX = (x / plotWidth) * svg.clientWidth;
      focus.setAttribute("x1", x);
      focus.setAttribute("x2", x);
      focus.style.display = "block";
      tooltip.textContent = `t=${payload.time[index].toFixed(3)} s | Adams ${formatValue(series.adams[index])} | native ${formatValue(series.native[index])} ${series.unit}`;
      tooltip.hidden = false;
      tooltip.style.left = `${Math.min(Math.max(cssX + 8, 4), Math.max(4, wrap.clientWidth - 250))}px`;
      tooltip.style.top = "6px";
    };
    hit.addEventListener("pointermove", showValue);
    hit.addEventListener("pointerleave", () => {
      tooltip.hidden = true;
      focus.style.display = "none";
    });
    return figure;
  };

  const groups = document.getElementById("comparison-groups");
  payload.groups.forEach(([title, names]) => {
    const section = document.createElement("section");
    section.className = "comparison-group";
    const heading = document.createElement("h2");
    heading.textContent = title;
    section.appendChild(heading);
    const grid = document.createElement("div");
    grid.className = "comparison-grid";
    names.forEach((name) => grid.appendChild(makePlot(name)));
    section.appendChild(grid);
    groups.appendChild(section);
  });
})();
</script>
""".replace("__PAYLOAD__", data)


def main() -> None:
    """Build the HTML fragment from one completed benchmark artifact."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=Path(
            "artifacts/real-adams-car/benchmark-equilibrium-wrench-00625ms/road_sine"
        ),
    )
    parser.add_argument(
        "--fragment",
        type=Path,
        default=Path(
            ".claude/visualizations/road-sine-adams-native-comparison.fragment.html"
        ),
    )
    args = parser.parse_args()
    payload = _load_payload(args.artifact_root)
    args.fragment.parent.mkdir(parents=True, exist_ok=True)
    args.fragment.write_text(_fragment(payload), encoding="utf-8")
    print(args.fragment)


if __name__ == "__main__":
    main()
