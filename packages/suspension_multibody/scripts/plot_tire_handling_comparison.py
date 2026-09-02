"""Generate a combined tire-force and handling comparison visual."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

TIRE_CHANNELS = (
    ("front_left.tire_normal_force", "左前轮 Fz"),
    ("front_right.tire_normal_force", "右前轮 Fz"),
    ("rear_left.tire_normal_force", "左后轮 Fz"),
    ("rear_right.tire_normal_force", "右后轮 Fz"),
    ("front_left.tire_longitudinal_force", "左前轮 Fx"),
    ("front_right.tire_longitudinal_force", "右前轮 Fx"),
    ("rear_left.tire_longitudinal_force", "左后轮 Fx"),
    ("rear_right.tire_longitudinal_force", "右后轮 Fx"),
    ("front_left.tire_lateral_force", "左前轮 Fy"),
    ("front_right.tire_lateral_force", "右前轮 Fy"),
    ("rear_left.tire_lateral_force", "左后轮 Fy"),
    ("rear_right.tire_lateral_force", "右后轮 Fy"),
)
HANDLING_CHANNELS = (
    ("lateral_acceleration", "横向加速度"),
    ("yaw_rate", "横摆角速度"),
    ("body_roll", "车身侧倾角"),
)
MODEL_ORDER = (
    "native_brush",
    "native_pac2002",
    "adams_pac2002",
)
MODEL_LABELS = {
    "native_brush": "native_brush",
    "native_pac2002": "native PAC2002",
    "adams_pac2002": "Adams PAC2002",
}


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return value


def _history_payload(
    path: Path,
    expected_model: str,
) -> tuple[list[float], dict[str, list[float]], dict[str, str]]:
    payload = _read(path)
    time = payload.get("time")
    channels = payload.get("channels")
    units = payload.get("units")
    metadata = payload.get("metadata")
    if not isinstance(metadata, dict):
        raise ValueError(f"历史文件缺少 metadata: {path}")
    if metadata.get("model_kind") != expected_model:
        raise ValueError(
            f"历史文件模型类型不匹配: {path}，"
            f"期望 {expected_model}，实际 {metadata.get('model_kind')}"
        )
    if metadata.get("complete_vehicle_reference") is not True:
        raise ValueError(f"历史文件不是完整整车模型结果: {path}")
    if not isinstance(time, list) or not isinstance(channels, dict):
        raise ValueError(f"历史文件缺少 time/channels: {path}")
    if not isinstance(units, dict):
        units = {}
    return (
        [float(value) for value in time],
        {str(name): [float(item) for item in values] for name, values in channels.items()},
        {str(name): str(unit) for name, unit in units.items()},
    )


def _check_grid(reference: list[float], candidate: list[float], label: str) -> None:
    if len(reference) != len(candidate) or any(
        abs(left - right) > 1.0e-12
        for left, right in zip(reference, candidate)
    ):
        raise ValueError(f"{label} 的 Adams/native 时间网格不一致")


def _metric(
    reference: list[float],
    actual: list[float],
    reference_model: str,
    actual_model: str,
) -> dict[str, Any]:
    reference_values = np.asarray(reference, dtype=float)
    actual_values = np.asarray(actual, dtype=float)
    error = actual_values - reference_values
    reference_peak = float(np.max(np.abs(reference_values)))
    actual_peak = float(np.max(np.abs(actual_values)))
    reference_rms = float(np.sqrt(np.mean(np.square(reference_values))))
    error_rms = float(np.sqrt(np.mean(np.square(error))))
    return {
        "nrmse": error_rms / max(reference_rms, np.finfo(float).eps),
        "maximum_absolute_error": float(np.max(np.abs(error))),
        "peak_relative_percent": 100.0
        * abs(actual_peak - reference_peak)
        / max(reference_peak, np.finfo(float).eps),
        "reference_peak": reference_peak,
        "actual_peak": actual_peak,
        "reference_rms": reference_rms,
        "error_rms": error_rms,
        "reference_model": reference_model,
        "actual_model": actual_model,
    }


def _series(
    histories: dict[str, tuple[list[float], dict[str, list[float]], dict[str, str]]],
    names: tuple[tuple[str, str], ...],
    label: str,
) -> dict[str, Any]:
    reference_time = histories["adams_pac2002"][0]
    result: dict[str, Any] = {
        "time": reference_time,
        "channels": {},
        "label": label,
    }
    for model_id, (time, _, _) in histories.items():
        _check_grid(reference_time, time, f"{label} · {MODEL_LABELS[model_id]}")
    for name, title in names:
        entries: dict[str, dict[str, Any]] = {}
        for model_id in MODEL_ORDER:
            _, channels, units = histories[model_id]
            if name not in channels:
                raise ValueError(f"{label} · {MODEL_LABELS[model_id]} 缺少通道: {name}")
            entries[model_id] = {
                "id": model_id,
                "label": MODEL_LABELS[model_id],
                "status": "available",
                "values": channels[name],
            }
        pair_metrics = {
            f"{actual}_vs_{reference}": _metric(
                histories[reference][1][name],
                histories[actual][1][name],
                reference,
                actual,
            )
            for reference, actual in (
                ("adams_pac2002", "native_pac2002"),
                ("adams_pac2002", "native_brush"),
                ("native_pac2002", "native_brush"),
            )
        }
        unit = histories["adams_pac2002"][2].get(name, "")
        metric_lines = []
        for reference, actual in (
            ("adams_pac2002", "native_pac2002"),
            ("adams_pac2002", "native_brush"),
            ("native_pac2002", "native_brush"),
        ):
            metric = pair_metrics[f"{actual}_vs_{reference}"]
            metric_lines.append(
                f"{MODEL_LABELS[reference]} ↔ {MODEL_LABELS[actual]} · "
                f"NRMSE {metric['nrmse'] * 100:.3f}% · "
                f"max {metric['maximum_absolute_error']:.6g} {unit}"
            )
        result["channels"][name] = {
            "title": title,
            "unit": unit,
            "models": [entries[model_id] for model_id in MODEL_ORDER if model_id in entries],
            "metric_text": "\n".join(metric_lines),
            "pair_metrics": pair_metrics,
        }
    return result


def _load_payload(comparison_root: Path) -> dict[str, Any]:
    comparison_root = comparison_root.resolve()
    histories = {
        "native_brush": _history_payload(
            comparison_root / "native_native_brush_time_history.json",
            "native_brush",
        ),
        "native_pac2002": _history_payload(
            comparison_root / "native_pac2002_time_history.json",
            "pac2002",
        ),
        "adams_pac2002": _history_payload(
            comparison_root / "adams_pac2002_time_history.json",
            "adams_pac2002",
        ),
    }
    reference_time, _, _ = histories["adams_pac2002"]
    for model_id, (time, channels, units) in histories.items():
        _check_grid(reference_time, time, f"三模型时间网格 · {MODEL_LABELS[model_id]}")
        if not units:
            raise ValueError(f"{MODEL_LABELS[model_id]} 缺少单位声明")
        if not channels:
            raise ValueError(f"{MODEL_LABELS[model_id]} 缺少历史通道")

    return {
        "tire": _series(
            histories,
            TIRE_CHANNELS,
            "完整 native 多体与 Adams PAC2002 轮胎力",
        ),
        "handling": _series(
            histories,
            HANDLING_CHANNELS,
            "完整 native 多体与 Adams PAC2002 操稳",
        ),
        "model_legend": [
            {"id": model_id, "label": MODEL_LABELS[model_id]}
            for model_id in MODEL_ORDER
        ],
        "model_matrix": {
            "tire": [
                {
                    "id": "native_brush",
                    "label": MODEL_LABELS["native_brush"],
                    "status": "可用",
                    "detail": "完整 native 多体，step_steer",
                },
                {
                    "id": "native_pac2002",
                    "label": MODEL_LABELS["native_pac2002"],
                    "status": "可用",
                    "detail": "完整 native 多体，step_steer",
                },
                {
                    "id": "adams_pac2002",
                    "label": MODEL_LABELS["adams_pac2002"],
                    "status": "可用",
                    "detail": "Adams/Car 原始 .res，step_steer",
                },
            ],
            "handling": [
                {
                    "id": "native_brush",
                    "label": MODEL_LABELS["native_brush"],
                    "status": "可用",
                    "detail": "完整 native 多体，step_steer",
                },
                {
                    "id": "native_pac2002",
                    "label": MODEL_LABELS["native_pac2002"],
                    "status": "可用",
                    "detail": "完整 native 多体，step_steer",
                },
                {
                    "id": "adams_pac2002",
                    "label": MODEL_LABELS["adams_pac2002"],
                    "status": "可用",
                    "detail": "Adams/Car 原始 .res，step_steer",
                },
            ],
        },
        "meta": {
            "tire_model": "完整 native 多体三模型",
            "tire_source": "三条曲线使用同一完整 step_steer 整车时间网格；native 两条由完整整车多体模型生成",
            "tire_duration_s": reference_time[-1],
            "handling_model": "完整 native 多体三模型",
            "handling_source": "三条曲线使用同一完整 step_steer 整车时间网格；native 两条由完整整车多体模型生成",
            "handling_case": "step_steer",
            "handling_duration_s": reference_time[-1],
            "handling_matrix_note": "native_brush 和 native PAC2002 均由 run_vehicle_dynamics 完整整车多体模型生成；Adams PAC2002 来自同一工况的原始 .res。页面同时给出三组曲线的两两误差；native_brush 与 Adams PAC2002 的误差不等同于同轮胎模型精度。",
            "comparison_root": str(comparison_root),
        },
    }


def _fragment(payload: dict[str, Any]) -> str:
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return f'''<div id="tire-handling-comparison">
  <div class="comparison-meta">
    <div>
      <strong>轮胎力：三模型对比矩阵</strong>
      <span class="muted">step_steer，{payload["meta"]["tire_duration_s"]:.3g} s</span>
    </div>
    <div>
      <strong>操稳：三模型对比矩阵</strong>
      <span class="muted">完整 native 多体与 Adams/Car</span>
    </div>
    <div id="model-legend" class="legend" aria-label="模型图例"></div>
  </div>
  <div class="matrix-grid" aria-label="三模型数据状态">
    <div class="matrix-panel">
      <strong>轮胎力数据状态</strong>
      <div id="tire-status" class="status-list"></div>
    </div>
    <div class="matrix-panel">
      <strong>操稳数据状态</strong>
      <div id="handling-status" class="status-list"></div>
    </div>
  </div>
  <section>
    <h2>轮胎力时间历程</h2>
    <div id="tire-plots" class="plot-grid"></div>
  </section>
  <section>
    <h2>操稳响应时间历程</h2>
    <div id="handling-plots" class="plot-grid"></div>
    <p class="muted note">{payload["meta"]["handling_matrix_note"]}</p>
  </section>
</div>
<style>
#tire-handling-comparison {{
  color: var(--foreground);
  background: transparent;
  font-family: var(--font-sans);
  font-size: var(--font-size-normal);
}}
#tire-handling-comparison * {{ box-sizing: border-box; }}
#tire-handling-comparison .comparison-meta {{
  display: flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: 10px 24px;
  margin-bottom: 16px;
}}
#tire-handling-comparison .comparison-meta > div {{
  display: flex;
  flex-direction: column;
  gap: 2px;
}}
#tire-handling-comparison .muted {{ color: var(--muted-foreground); }}
#tire-handling-comparison .legend {{
  display: flex;
  flex-wrap: wrap;
  flex-direction: row !important;
  gap: 12px !important;
  margin-left: auto;
}}
#tire-handling-comparison .legend span {{ white-space: nowrap; }}
#tire-handling-comparison .line {{
  display: inline-block;
  width: 18px;
  margin-right: 5px;
  vertical-align: middle;
  border-top: 2px solid var(--viz-series-1);
}}
#tire-handling-comparison .line.native-brush {{
  border-top-color: var(--viz-series-3);
  border-top-style: dotted;
}}
#tire-handling-comparison .line.native-pac2002 {{
  border-top-color: var(--viz-series-2);
  border-top-style: dashed;
}}
#tire-handling-comparison .line.adams-pac2002 {{
  border-top-color: var(--viz-series-1);
  border-top-style: solid;
}}
#tire-handling-comparison .matrix-grid {{
  display: grid;
  grid-template-columns: repeat(2, minmax(260px, 1fr));
  gap: 10px 18px;
  margin-bottom: 16px;
}}
#tire-handling-comparison .matrix-panel {{
  min-width: 0;
  padding: 9px 10px;
  border: 1px solid var(--border);
  border-radius: var(--radius-sm);
}}
#tire-handling-comparison .status-list {{
  display: grid;
  gap: 4px;
  margin-top: 6px;
}}
#tire-handling-comparison .status-item {{
  display: grid;
  grid-template-columns: minmax(120px, max-content) auto 1fr;
  gap: 7px;
  align-items: baseline;
  font-size: var(--font-size-small);
}}
#tire-handling-comparison .status-badge {{
  color: var(--muted-foreground);
  font-size: var(--font-size-small);
}}
#tire-handling-comparison .status-item.blocked .status-badge {{
  color: var(--destructive);
}}
#tire-handling-comparison section {{ margin-top: 18px; }}
#tire-handling-comparison h2 {{
  margin: 0 0 8px;
  font-size: var(--font-size-h3);
  font-weight: var(--font-weight-medium);
}}
#tire-handling-comparison .plot-grid {{
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  gap: 16px 18px;
}}
#tire-handling-comparison figure {{ margin: 0; min-width: 0; }}
#tire-handling-comparison figcaption {{
  display: grid;
  gap: 2px;
  margin-bottom: 3px;
  overflow-wrap: anywhere;
}}
#tire-handling-comparison .plot-title {{
  font-weight: var(--font-weight-medium);
  white-space: nowrap;
}}
#tire-handling-comparison .plot-metric {{
  color: var(--muted-foreground);
  font-size: var(--font-size-small);
  line-height: 1.25;
  white-space: pre-line;
}}
#tire-handling-comparison svg {{
  display: block;
  width: 100%;
  height: auto;
  overflow: visible;
}}
#tire-handling-comparison .grid-line {{ stroke: var(--border); stroke-width: 1; }}
#tire-handling-comparison .zero-line {{ stroke: var(--muted-foreground); stroke-width: 1; stroke-dasharray: 2 3; opacity: .65; }}
#tire-handling-comparison .axis-label {{ fill: var(--muted-foreground); font-size: 10px; }}
#tire-handling-comparison .plot-native-brush {{ fill: none; stroke: var(--viz-series-3); stroke-width: 1.4; stroke-dasharray: 1 3; }}
#tire-handling-comparison .plot-native-pac2002 {{ fill: none; stroke: var(--viz-series-2); stroke-width: 1.4; stroke-dasharray: 5 3; }}
#tire-handling-comparison .plot-adams-pac2002 {{ fill: none; stroke: var(--viz-series-1); stroke-width: 1.6; }}
#tire-handling-comparison .blocked-plot {{
  min-height: 54px;
  padding: 12px;
  border: 1px dashed var(--border);
  color: var(--muted-foreground);
  font-size: var(--font-size-small);
}}
#tire-handling-comparison .plot-hit {{ fill: transparent; cursor: crosshair; }}
#tire-handling-comparison .focus {{ fill: var(--foreground); stroke: var(--background); stroke-width: 1.5; }}
#tire-handling-comparison .tooltip {{
  position: fixed;
  z-index: 10;
  padding: 4px 7px;
  border: 1px solid var(--border);
  border-radius: var(--radius-sm);
  color: var(--popover-foreground);
  background: var(--popover);
  font-size: var(--font-size-small);
  pointer-events: none;
  white-space: nowrap;
}}
#tire-handling-comparison .note {{ margin: 8px 0 0; font-size: var(--font-size-small); }}
@media (max-width: 560px) {{
  #tire-handling-comparison .legend {{ margin-left: 0; }}
  #tire-handling-comparison .matrix-grid {{ grid-template-columns: 1fr; }}
  #tire-handling-comparison .status-item {{ grid-template-columns: 1fr auto; }}
  #tire-handling-comparison .status-detail {{ grid-column: 1 / -1; }}
  #tire-handling-comparison .plot-grid {{ grid-template-columns: 1fr; }}
}}
</style>
<script>
(() => {{
  const payload = {data};
  const root = document.getElementById("tire-handling-comparison");
  if (!root) return;
  root.querySelector(".note").textContent = payload.meta.handling_matrix_note;
  const NS = "http://www.w3.org/2000/svg";
  const make = (tag, attrs = {{}}) => {{
    const node = document.createElementNS(NS, tag);
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
    return node;
  }};
  const text = (tag, attrs, value) => {{
    const node = make(tag, attrs);
    node.textContent = value;
    return node;
  }};
  const number = (value) => {{
    const abs = Math.abs(value);
    if (abs !== 0 && (abs < 1e-3 || abs >= 1e4)) return value.toExponential(2);
    return value.toFixed(abs < 1 ? 5 : 2);
  }};
  const modelClass = {{
    native_brush: "native-brush",
    native_pac2002: "native-pac2002",
    adams_pac2002: "adams-pac2002",
  }};
  const legend = root.querySelector("#model-legend");
  payload.model_legend.forEach((model) => {{
    const item = document.createElement("span");
    const line = document.createElement("i");
    line.className = `line ${{modelClass[model.id] || ""}}`;
    line.setAttribute("aria-hidden", "true");
    item.appendChild(line);
    item.appendChild(document.createTextNode(model.label));
    legend.appendChild(item);
  }});
  const renderStatus = (name, host) => {{
    payload.model_matrix[name].forEach((model) => {{
      const item = document.createElement("div");
      item.className = `status-item ${{model.status === "阻塞" ? "blocked" : "available"}}`;
      const label = document.createElement("strong");
      label.textContent = model.label;
      const badge = document.createElement("span");
      badge.className = "status-badge";
      badge.textContent = model.status;
      const detail = document.createElement("span");
      detail.className = "status-detail muted";
      detail.textContent = model.detail;
      item.append(label, badge, detail);
      host.appendChild(item);
    }});
  }};
  renderStatus("tire", root.querySelector("#tire-status"));
  renderStatus("handling", root.querySelector("#handling-status"));
  const plot = (series, name, host) => {{
    const values = series.channels[name];
    const time = series.time;
    const available = values.models.filter((model) => model.status === "available");
    const all = available.flatMap((model) => model.values);
    if (!available.length) {{
      const figure = document.createElement("figure");
      const caption = document.createElement("figcaption");
      const title = document.createElement("span");
      title.className = "plot-title";
      title.textContent = values.title;
      caption.appendChild(title);
      const blocked = document.createElement("div");
      blocked.className = "blocked-plot";
      blocked.textContent = values.models.map((model) => `${{model.label}}：${{model.reason || "无数据"}}`).join("；");
      figure.append(caption, blocked);
      host.appendChild(figure);
      return;
    }}
    let low = Math.min(...all);
    let high = Math.max(...all);
    if (low === high) {{ low -= 1; high += 1; }}
    const pad = (high - low) * .08;
    low -= pad; high += pad;
    const width = 640, height = 220;
    const left = 48, right = 10, top = 10, bottom = 28;
    const innerW = width - left - right, innerH = height - top - bottom;
    const x = (i) => left + i * innerW / Math.max(1, time.length - 1);
    const y = (v) => top + (high - v) * innerH / (high - low);
    const path = (data) => data.map((value, i) => `${{i ? "L" : "M"}}${{x(i).toFixed(2)}},${{y(value).toFixed(2)}}`).join(" ");
    const figure = document.createElement("figure");
    const caption = document.createElement("figcaption");
    const title = document.createElement("span");
    title.className = "plot-title";
    title.textContent = values.title;
    const metric = document.createElement("span");
    metric.className = "plot-metric";
    metric.textContent = values.metric_text;
    caption.append(title, metric);
    figure.appendChild(caption);
    const labels = available.map((model) => model.label).join("、");
    const svg = make("svg", {{ viewBox: `0 0 ${{width}} ${{height}}`, role: "img", "aria-label": `${{values.title}} ${{labels}} 对比` }});
    const desc = document.createElementNS(NS, "desc");
    desc.textContent = `${{labels}} 的 ${{values.title}} 时间历程，三套历史使用同一输出时间网格。`;
    svg.appendChild(desc);
    [0, .5, 1].forEach((fraction) => {{
      const yy = top + fraction * innerH;
      svg.appendChild(make("line", {{ class: "grid-line", x1: left, x2: width - right, y1: yy, y2: yy }}));
    }});
    if (low < 0 && high > 0) svg.appendChild(make("line", {{ class: "zero-line", x1: left, x2: width - right, y1: y(0), y2: y(0) }}));
    svg.appendChild(text("text", {{ class: "axis-label", x: left - 5, y: top + 4, "text-anchor": "end" }}, number(high)));
    svg.appendChild(text("text", {{ class: "axis-label", x: left - 5, y: top + innerH / 2 + 4, "text-anchor": "end" }}, number((high + low) / 2)));
    svg.appendChild(text("text", {{ class: "axis-label", x: left - 5, y: top + innerH + 4, "text-anchor": "end" }}, number(low)));
    [0, Math.floor((time.length - 1) / 2), time.length - 1].forEach((index) => svg.appendChild(text("text", {{ class: "axis-label", x: x(index), y: height - 7, "text-anchor": index === 0 ? "start" : index === time.length - 1 ? "end" : "middle" }}, `${{time[index].toFixed(3)}} s`)));
    available.forEach((model) => svg.appendChild(make("path", {{ class: `plot-${{modelClass[model.id] || "unknown"}}`, d: path(model.values) }})));
    const focus = make("circle", {{ class: "focus", r: 3.5, cx: left, cy: y(available[0].values[0]) }});
    focus.style.display = "none";
    svg.appendChild(focus);
    const hit = make("rect", {{ class: "plot-hit", x: left, y: top, width: innerW, height: innerH }});
    svg.appendChild(hit);
    const tooltip = document.createElement("div");
    tooltip.className = "tooltip";
    tooltip.hidden = true;
    figure.appendChild(svg);
    figure.appendChild(tooltip);
    const show = (event) => {{
      const box = svg.getBoundingClientRect();
      const localX = Math.max(0, Math.min(box.width, event.clientX - box.left));
      const index = Math.max(0, Math.min(time.length - 1, Math.round(localX / box.width * (time.length - 1))));
      const px = left + index * innerW / Math.max(1, time.length - 1);
      focus.setAttribute("cx", px);
      focus.setAttribute("cy", y(available[0].values[index]));
      focus.style.display = "block";
      tooltip.hidden = false;
      const readings = available.map((model) => `${{model.label}} ${{number(model.values[index])}}`).join(" · ");
      tooltip.textContent = `t=${{time[index].toFixed(3)}} s · ${{readings}} ${{values.unit}}`;
      const leftPx = Math.min(window.innerWidth - tooltip.offsetWidth - 6, Math.max(6, event.clientX + 8));
      tooltip.style.left = `${{leftPx}}px`;
      tooltip.style.top = `${{Math.max(6, event.clientY - tooltip.offsetHeight - 8)}}px`;
    }};
    hit.addEventListener("pointermove", show);
    hit.addEventListener("pointerleave", () => {{ tooltip.hidden = true; focus.style.display = "none"; }});
    host.appendChild(figure);
  }};
  Object.keys(payload.tire.channels).forEach((name) => plot(payload.tire, name, root.querySelector("#tire-plots")));
  Object.keys(payload.handling.channels).forEach((name) => plot(payload.handling, name, root.querySelector("#handling-plots")));
}})();
</script>'''.replace("{{NOTE}}", payload["meta"]["handling_matrix_note"])


def main() -> None:
    """Generate the standalone tire-force and handling comparison page."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--comparison-root",
        type=Path,
        default=Path("artifacts/visuals/full-native-three-model-step-steer/step_steer"),
    )
    parser.add_argument(
        "--fragment",
        type=Path,
        default=Path(".claude/visualizations/tire-handling-comparison.fragment.html"),
    )
    args = parser.parse_args()
    payload = _load_payload(args.comparison_root)
    args.fragment.parent.mkdir(parents=True, exist_ok=True)
    args.fragment.write_text(_fragment(payload), encoding="utf-8")
    print(args.fragment)


if __name__ == "__main__":
    main()
