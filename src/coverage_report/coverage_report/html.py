"""Render the run report as one self-contained HTML page (inline SVG, no JS, no deps)."""

from __future__ import annotations

import bisect
import html
from typing import Dict, List, Sequence, Tuple

from .metrics import RunData

SPRAY = "#1f6feb"    # valve open
TRAVEL = "#9aa4ae"   # valve closed
TARGET = "#e8eef5"   # cells to treat
FAULT = "#d1242f"


def _fmt(v, unit="", digits=1):
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{digits}f}{unit}"
    return f"{v}{unit}"


def _trajectory_svg(run: RunData, events: Sequence[Dict], t0: float, width: int = 640) -> str:
    """Top view: target cells, planned path, driven path coloured by valve state, faults."""
    w_m = run.grid_width * run.resolution or 1.0
    h_m = run.grid_height * run.resolution or 1.0
    pad = 10
    s = (width - 2 * pad) / w_m
    height = int(h_m * s + 2 * pad)

    def px(x: float, y: float) -> Tuple[float, float]:  # metres -> SVG (y up)
        return pad + x * s, height - pad - y * s

    parts: List[str] = [f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
                        'xmlns="http://www.w3.org/2000/svg" role="img" aria-label="trajectory">']
    c = run.resolution * s
    for r in range(run.grid_height):
        for col in range(run.grid_width):
            if run.target[r * run.grid_width + col]:
                x, y = px(col * run.resolution, (r + 1) * run.resolution)
                parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{c:.2f}" height="{c:.2f}" '
                             f'fill="{TARGET}"/>')
    if run.path:
        pts = " ".join("%.1f,%.1f" % px(x, y) for x, y, _ in run.path)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#555" stroke-width="1" '
                     'stroke-dasharray="4 3"/>')
    # driven path: one polyline per run of equal valve state
    seg: List[str] = []
    seg_spray = None
    spray_times = [s[0] for s in run.spray]
    for t, x, y in run.true_poses:
        i = bisect.bisect_right(spray_times, t) - 1
        on = i >= 0 and run.spray[i][1]
        if seg_spray is not None and on != seg_spray:
            seg.append("%.1f,%.1f" % px(x, y))
            parts.append(_poly(seg, seg_spray))
            seg = []
        seg_spray = on
        seg.append("%.1f,%.1f" % px(x, y))
    if len(seg) > 1:
        parts.append(_poly(seg, bool(seg_spray)))
    # fault / e-stop markers at the true position at that moment
    for e in events:
        if e["state"].startswith(("FAULT", "ESTOP")):
            t = t0 + e["t_s"]
            pose = min(run.true_poses, key=lambda p: abs(p[0] - t), default=None)
            if pose:
                x, y = px(pose[1], pose[2])
                parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" fill="none" '
                             f'stroke="{FAULT}" stroke-width="2"><title>'
                             f'{html.escape(e["state"])} at {e["t_s"]:.1f} s</title></circle>')
    parts.append("</svg>")
    return "".join(parts)


def _poly(points: List[str], spray: bool) -> str:
    colour, w = (SPRAY, 3) if spray else (TRAVEL, 1.5)
    return (f'<polyline points="{" ".join(points)}" fill="none" stroke="{colour}" '
            f'stroke-width="{w}" stroke-linejoin="round"/>')


def _coverage_svg(coverage: Sequence[Tuple[float, float]], t0: float, width: int = 640,
                  height: int = 160) -> str:
    """Coverage % over time."""
    if not coverage:
        return "<p>No coverage data.</p>"
    pad_l, pad_b, pad = 36, 20, 8
    t_max = max(t - t0 for t, _ in coverage) or 1.0
    sx = (width - pad_l - pad) / t_max
    sy = (height - pad_b - pad) / 100.0
    pts = " ".join(f"{pad_l + max(0.0, t - t0) * sx:.1f},{height - pad_b - p * sy:.1f}"
                   for t, p in coverage)
    axis = (f'<line x1="{pad_l}" y1="{height - pad_b}" x2="{width - pad}" y2="{height - pad_b}" '
            'stroke="#999"/>'
            f'<line x1="{pad_l}" y1="{pad}" x2="{pad_l}" y2="{height - pad_b}" stroke="#999"/>'
            f'<text x="{pad_l - 4}" y="{pad + 4}" font-size="10" text-anchor="end">100%</text>'
            f'<text x="{pad_l - 4}" y="{height - pad_b}" font-size="10" text-anchor="end">0%</text>'
            f'<text x="{width - pad}" y="{height - 4}" font-size="10" text-anchor="end">'
            f'{t_max:.0f} s</text>')
    return (f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
            f'xmlns="http://www.w3.org/2000/svg" role="img" aria-label="coverage over time">{axis}'
            f'<polyline points="{pts}" fill="none" stroke="{SPRAY}" stroke-width="2"/></svg>')


def render(run: RunData, m: Dict, title: str) -> str:
    events = m["events"]
    t0 = m["start_time"]  # event times are relative to this (first RUNNING)
    s = m["sensor"]
    rows = [
        ("Completed", _fmt(m["completed"])),
        ("Final coverage", _fmt(m["final_coverage_pct"], " %")),
        ("Duration (start to complete)", _fmt(m["duration_s"], " s")),
        ("Target area", _fmt(m["target_area_m2"], " m²", 2) + f' ({m["target_cells"]} cells)'),
        ("Planned path", _fmt(m["planned_path_m"], " m", 2)),
        ("Sprayed outside the marked area (overspray)", _fmt(m["overspray_m2"], " m²", 3)),
        ("Driven while spraying / travelling",
         _fmt(m["driven_spraying_m"], " m", 2) + " / " + _fmt(m["driven_travelling_m"], " m", 2)),
        ("E-stops / pauses", f'{m["estops"]} / {m["pauses"]}'),
        ("Faults (recovered)", f'{m["faults"]} ({m["recoveries"]})'
         + (" – " + html.escape(", ".join(m["fault_reasons"])) if m["fault_reasons"] else "")),
        ("Sensor messages", str(s["messages"])),
        ("Sensor: longest silence", _fmt(s["longest_gap_s"], " s", 2)
         + f' ({s["gaps_over_threshold"]} gaps &gt; {s["gap_threshold_s"]:.1f} s)'),
        ("Sensor error when measured (RMS, noise only)", _fmt(s["rms_error_mm"], " mm")),
        ("Sensor error when it arrives (RMS, noise + latency)",
         _fmt(s["rms_error_at_arrival_mm"], " mm")),
        ("Controller estimate error (RMS, after fusion + latency compensation)",
         _fmt(m["estimate_rms_error_mm"], " mm")),
    ]
    table = "".join(f"<tr><th>{k}</th><td>{v}</td></tr>" for k, v in rows)
    ev = "".join(f'<tr><td>{e["t_s"]:.2f} s</td><td>{html.escape(e["state"])}</td></tr>'
                 for e in events)
    status = "ok" if m["completed"] and m["final_coverage_pct"] >= 99.0 else "warn"
    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>
 body{{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;max-width:720px;margin:2rem auto;
      padding:0 1rem;color:#1f2328;line-height:1.45}}
 h1{{font-size:1.4rem;margin-bottom:.2rem}} h2{{font-size:1.05rem;margin-top:1.8rem}}
 .note{{color:#57606a;font-size:.85rem}} table{{border-collapse:collapse;width:100%;font-size:.92rem}}
 th,td{{text-align:left;padding:.3rem .5rem;border-bottom:1px solid #eaeef2;vertical-align:top}}
 th{{font-weight:600;width:55%}} .badge{{display:inline-block;padding:.1rem .5rem;border-radius:1rem;
 font-size:.8rem;font-weight:600}} .ok{{background:#dafbe1;color:#116329}}
 .warn{{background:#fff1e5;color:#953800}} .legend span{{margin-right:1rem}}
</style></head><body>
<h1>{html.escape(title)} <span class="badge {status}">{_fmt(m["final_coverage_pct"], " %")} covered</span></h1>
<p class="note">Simulation-only run of the scaled coverage robot. Coverage is measured on the
simulator's ground-truth nozzle position; the controller only saw the (possibly faulty) sensor.</p>
<h2>Summary</h2><table>{table}</table>
<h2>Top view</h2>
<p class="legend note"><span style="color:{SPRAY}">━ spraying</span><span style="color:{TRAVEL}">━ travelling</span>
<span>┅ planned</span><span style="color:{FAULT}">○ fault / e-stop</span></p>
{_trajectory_svg(run, events, t0)}
<h2>Coverage over time</h2>
{_coverage_svg(run.coverage, t0)}
<h2>State changes</h2><table><tr><th>Time (from start)</th><th>State</th></tr>{ev}</table>
</body></html>
"""
