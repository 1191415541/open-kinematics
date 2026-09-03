"""Generate a Native Fiala versus Adams Fiala handling comparison page."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


CHANNELS = (
    ("lateral_acceleration", "横向加速度"),
    ("yaw_rate", "横摆角速度"),
    ("body_roll", "车身侧倾角"),
)


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return value


def _metric(reference: list[float], actual: list[float]) -> dict[str, float]:
    ref = np.asarray(reference, dtype=float)
    act = np.asarray(actual, dtype=float)
    error = act - ref
    return {
        "nrmse_percent": float(
            100.0 * np.sqrt(np.mean(np.square(error))) / max(np.sqrt(np.mean(np.square(ref))), np.finfo(float).eps)
        ),
        "max_abs_error": float(np.max(np.abs(error))),
    }


def _load_payload(root: Path) -> dict[str, Any]:
    adams = _read(root / "adams_fiala_time_history.json")
    native = _read(root / "native_fiala_time_history.json")
    time = adams.get("time")
    native_time = native.get("time")
    if not isinstance(time, list) or not isinstance(native_time, list):
        raise ValueError("历史文件缺少 time 数组")
    if len(time) != len(native_time) or any(abs(float(a) - float(b)) > 1e-12 for a, b in zip(time, native_time)):
        raise ValueError("Adams 与 native 的时间网格不一致")
    adams_channels = adams.get("channels")
    native_channels = native.get("channels")
    adams_units = adams.get("units") or {}
    if not isinstance(adams_channels, dict) or not isinstance(native_channels, dict):
        raise ValueError("历史文件缺少 channels 对象")

    channels: dict[str, Any] = {}
    for name, title in CHANNELS:
        reference = adams_channels.get(name)
        actual = native_channels.get(name)
        if not isinstance(reference, list) or not isinstance(actual, list):
            raise ValueError(f"缺少操稳通道: {name}")
        if len(reference) != len(time) or len(actual) != len(time):
            raise ValueError(f"通道长度不匹配: {name}")
        channels[name] = {
            "title": title,
            "unit": str(adams_units.get(name, "")),
            "adams": [float(value) for value in reference],
            "native": [float(value) for value in actual],
            "metric": _metric(reference, actual),
        }
    return {
        "time": [float(value) for value in time],
        "channels": channels,
        "meta": {
            "duration_s": float(time[-1]),
            "sample_count": len(time),
            "step_s": float(time[1] - time[0]) if len(time) > 1 else 0.0,
        },
    }


def _fragment(payload: dict[str, Any]) -> str:
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return f'''<div id="native-fiala-handling-comparison">
  <p class="context">完整 Native 多体 Fiala 与 Adams Fiala · step_steer · 公共 {payload["meta"]["step_s"]:.3g} s 网格 · {payload["meta"]["duration_s"]:.3g} s</p>
  <div class="legend" aria-label="模型图例">
    <span><i class="line adams" aria-hidden="true"></i>Adams Fiala</span>
    <span><i class="line native" aria-hidden="true"></i>Native Fiala</span>
  </div>
  <div id="plots" class="plots"></div>
</div>
<style>
#native-fiala-handling-comparison {{ color: var(--foreground); width: 100%; font-family: inherit; }}
#native-fiala-handling-comparison .context {{ margin: 0 0 .5rem; color: var(--muted-foreground); }}
#native-fiala-handling-comparison .legend {{ display: flex; flex-wrap: wrap; gap: .75rem 1.2rem; color: var(--muted-foreground); font-size: var(--font-size-small); margin-bottom: 1rem; }}
#native-fiala-handling-comparison .legend span {{ display: inline-flex; align-items: center; gap: .35rem; }}
#native-fiala-handling-comparison .line {{ width: 1.8rem; height: 0; border-top: 2px solid var(--viz-series-1); }}
#native-fiala-handling-comparison .line.native {{ border-top-color: var(--viz-series-2); border-top-style: dashed; }}
#native-fiala-handling-comparison .plots {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 1rem; }}
#native-fiala-handling-comparison figure {{ margin: 0; min-width: 0; }}
#native-fiala-handling-comparison figcaption {{ display: flex; flex-wrap: wrap; justify-content: space-between; gap: .25rem .5rem; margin-bottom: .25rem; }}
#native-fiala-handling-comparison .title {{ font-weight: 500; }}
#native-fiala-handling-comparison .metric {{ color: var(--muted-foreground); font-size: var(--font-size-small); font-variant-numeric: tabular-nums; }}
#native-fiala-handling-comparison svg {{ display: block; width: 100%; height: auto; overflow: visible; }}
#native-fiala-handling-comparison .grid {{ stroke: var(--border); stroke-width: 1; vector-effect: non-scaling-stroke; }}
#native-fiala-handling-comparison .zero {{ stroke: var(--muted-foreground); stroke-dasharray: 2 3; stroke-width: 1; vector-effect: non-scaling-stroke; }}
#native-fiala-handling-comparison .adams {{ fill: none; stroke: var(--viz-series-1); stroke-width: 1.7; vector-effect: non-scaling-stroke; }}
#native-fiala-handling-comparison .native {{ fill: none; stroke: var(--viz-series-2); stroke-width: 1.5; stroke-dasharray: 5 3; vector-effect: non-scaling-stroke; }}
#native-fiala-handling-comparison .axis {{ fill: var(--muted-foreground); font-size: var(--font-size-small); font-variant-numeric: tabular-nums; }}
#native-fiala-handling-comparison .hit {{ fill: transparent; cursor: crosshair; }}
#native-fiala-handling-comparison .focus {{ stroke: var(--muted-foreground); stroke-dasharray: 3 3; vector-effect: non-scaling-stroke; }}
#native-fiala-handling-comparison .tooltip {{ position: fixed; z-index: 2; padding: .3rem .45rem; border: 1px solid var(--border); background: var(--popover); color: var(--popover-foreground); font-size: var(--font-size-small); pointer-events: none; white-space: nowrap; }}
@media (max-width: 680px) {{ #native-fiala-handling-comparison .plots {{ grid-template-columns: 1fr; }} }}
</style>
<script>
(() => {{
  const payload = {data};
  const root = document.getElementById("native-fiala-handling-comparison");
  if (!root) return;
  const NS = "http://www.w3.org/2000/svg";
  const fmt = (v) => {{ const a = Math.abs(v); return a && (a < 1e-3 || a >= 1e4) ? v.toExponential(3) : v.toPrecision(5); }};
  const node = (tag, attrs) => {{ const n = document.createElementNS(NS, tag); Object.entries(attrs).forEach(([k, v]) => n.setAttribute(k, v)); return n; }};
  Object.entries(payload.channels).forEach(([name, series]) => {{
    const all = series.adams.concat(series.native);
    let low = Math.min(...all), high = Math.max(...all);
    if (low === high) {{ low -= 1; high += 1; }}
    const pad = Math.max((high - low) * .08, Math.max(Math.abs(low), Math.abs(high)) * .02, 1e-12);
    low -= pad; high += pad;
    const w = 520, h = 250, left = 58, right = 12, top = 12, bottom = 30;
    const iw = w - left - right, ih = h - top - bottom;
    const x = (i) => left + i * iw / Math.max(1, payload.time.length - 1);
    const y = (v) => top + (high - v) * ih / (high - low);
    const path = (values) => values.map((v, i) => `${{i ? "L" : "M"}}${{x(i).toFixed(2)}} ${{y(v).toFixed(2)}}`).join(" ");
    const figure = document.createElement("figure");
    const caption = document.createElement("figcaption");
    const title = document.createElement("span"); title.className = "title"; title.textContent = `${{series.title}} (${{series.unit}})`;
    const metric = document.createElement("span"); metric.className = "metric"; metric.textContent = `NRMSE ${{series.metric.nrmse_percent.toFixed(3)}}% · max ${{fmt(series.metric.max_abs_error)}} ${{series.unit}}`;
    caption.append(title, metric); figure.appendChild(caption);
    const svg = node("svg", {{ viewBox: `0 0 ${{w}} ${{h}}`, role: "img", "aria-label": `${{series.title}}：Adams Fiala 与 Native Fiala 对比` }});
    [0, .5, 1].forEach((f) => {{ const yy = top + f * ih; svg.appendChild(node("line", {{ class: f === .5 && low < 0 && high > 0 ? "zero" : "grid", x1: left, x2: w - right, y1: yy, y2: yy }})); svg.appendChild(Object.assign(node("text", {{ class: "axis", x: left - 6, y: yy + 4, "text-anchor": "end" }}), {{ textContent: fmt(high - f * (high - low)) }})); }});
    [0, .5, 1].forEach((f) => {{ const xx = left + f * iw; svg.appendChild(node("line", {{ class: "grid", x1: xx, x2: xx, y1: top, y2: top + ih }})); svg.appendChild(Object.assign(node("text", {{ class: "axis", x: xx, y: h - 8, "text-anchor": f === 0 ? "start" : f === 1 ? "end" : "middle" }}), {{ textContent: `${{(payload.time[0] + f * (payload.time[payload.time.length - 1] - payload.time[0])).toFixed(2)}} s` }})); }});
    svg.appendChild(node("path", {{ class: "adams", d: path(series.adams) }}));
    svg.appendChild(node("path", {{ class: "native", d: path(series.native) }}));
    const focus = node("line", {{ class: "focus", x1: left, x2: left, y1: top, y2: top + ih }}); focus.style.display = "none"; svg.appendChild(focus);
    const hit = node("rect", {{ class: "hit", x: left, y: top, width: iw, height: ih }}); svg.appendChild(hit);
    const tooltip = document.createElement("div"); tooltip.className = "tooltip"; tooltip.hidden = true; figure.append(svg, tooltip);
    hit.addEventListener("pointermove", (event) => {{ const box = svg.getBoundingClientRect(); const fraction = Math.max(0, Math.min(1, (event.clientX - box.left) / box.width)); const i = Math.round(fraction * (payload.time.length - 1)); const xx = x(i); focus.setAttribute("x1", xx); focus.setAttribute("x2", xx); focus.style.display = "block"; tooltip.textContent = `t=${{payload.time[i].toFixed(3)}} s · Adams ${{fmt(series.adams[i])}} · Native ${{fmt(series.native[i])}} ${{series.unit}}`; tooltip.hidden = false; tooltip.style.left = `${{Math.min(window.innerWidth - tooltip.offsetWidth - 8, event.clientX + 8)}}px`; tooltip.style.top = `${{Math.max(6, event.clientY - tooltip.offsetHeight - 8)}}px`; }});
    hit.addEventListener("pointerleave", () => {{ tooltip.hidden = true; focus.style.display = "none"; }});
    root.querySelector("#plots").appendChild(figure);
  }});
}})();
</script>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison-root", type=Path, required=True)
    parser.add_argument("--fragment", type=Path, required=True)
    args = parser.parse_args()
    payload = _load_payload(args.comparison_root.resolve())
    args.fragment.parent.mkdir(parents=True, exist_ok=True)
    args.fragment.write_text(_fragment(payload), encoding="utf-8")
    print(args.fragment)


if __name__ == "__main__":
    main()
