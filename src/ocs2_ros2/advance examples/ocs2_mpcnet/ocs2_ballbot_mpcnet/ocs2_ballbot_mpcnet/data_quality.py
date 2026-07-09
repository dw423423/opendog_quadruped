#!/usr/bin/env python3

"""Generate a lightweight MPC-Net data quality report for ballbot.

The script runs one MPC-Net data generation pass and writes a self-contained
HTML report, a JSON summary, and the raw arrays as an NPZ file. It intentionally
does not depend on matplotlib so it can run in the existing MPC-Net venv.
"""

import argparse
import datetime
import html
import json
import os
from pathlib import Path
import tempfile
import time
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import torch
import torch.onnx

from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.policy import LinearPolicy

from ocs2_ballbot_mpcnet import MpcnetInterface
from ocs2_ballbot_mpcnet.mpcnet import BallbotMpcnet


STATE_LABELS = [
    "px",
    "py",
    "yaw",
    "pitch",
    "roll",
    "vx",
    "vy",
    "yaw_rate",
    "pitch_rate",
    "roll_rate",
]
INPUT_LABELS = ["torque_0", "torque_1", "torque_2"]


def parse_args() -> argparse.Namespace:
    package_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(package_dir / "config" / "ballbot.yaml"))
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--policy-file", default=None, help="Existing ONNX policy file. If omitted, a temporary linear policy is exported.")
    parser.add_argument("--alpha", type=float, default=1.0, help="Behavioral controller MPC weight. 1.0 means pure MPC rollout.")
    parser.add_argument("--tasks", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--time-step", type=float, default=None)
    parser.add_argument("--data-decimation", type=int, default=None)
    parser.add_argument("--samples", type=int, default=None)
    parser.add_argument("--threads", type=int, default=None)
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
    return parser.parse_args()


def make_output_dir(package_dir: Path, output_dir: Optional[str]) -> Path:
    if output_dir:
        out = Path(output_dir).expanduser().resolve()
    else:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out = package_dir / "runs" / f"data_quality_{timestamp}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def make_task_factory(config: Config) -> BallbotMpcnet:
    task_factory = object.__new__(BallbotMpcnet)
    task_factory.config = config
    return task_factory


def make_policy_file(config: Config, provided_policy_file: Optional[str]) -> Tuple[str, Optional[tempfile.TemporaryDirectory]]:
    if provided_policy_file:
        policy_path = Path(provided_policy_file).expanduser().resolve()
        if not policy_path.is_file():
            raise FileNotFoundError(f"policy file does not exist: {policy_path}")
        return str(policy_path), None

    temp_dir = tempfile.TemporaryDirectory(prefix="ballbot_data_quality_")
    policy_path = Path(temp_dir.name) / "diagnostic_policy.onnx"
    policy = LinearPolicy(config).to(config.DEVICE)
    dummy_observation = torch.randn(1, config.OBSERVATION_DIM, device=config.DEVICE, dtype=config.DTYPE)
    torch.onnx.export(
        model=policy,
        args=dummy_observation,
        f=str(policy_path),
        dynamo=False,
        opset_version=11,
    )
    return str(policy_path), temp_dir


def run_data_generation(config: Config, args: argparse.Namespace) -> List[object]:
    tasks = args.tasks if args.tasks is not None else config.DATA_GENERATION_TASKS
    duration = args.duration if args.duration is not None else config.DATA_GENERATION_DURATION
    time_step = args.time_step if args.time_step is not None else config.DATA_GENERATION_TIME_STEP
    data_decimation = args.data_decimation if args.data_decimation is not None else config.DATA_GENERATION_DATA_DECIMATION
    samples = args.samples if args.samples is not None else config.DATA_GENERATION_SAMPLES
    threads = args.threads if args.threads is not None else config.DATA_GENERATION_THREADS

    task_factory = make_task_factory(config)
    initial_observations, mode_schedules, target_trajectories = task_factory.get_tasks(tasks, duration)
    sampling_covariance = np.diag(np.power(np.asarray(config.DATA_GENERATION_SAMPLING_VARIANCE), 2))

    interface = MpcnetInterface(threads, 0, config.RAISIM)
    policy_file, temp_dir = make_policy_file(config, args.policy_file)
    try:
        interface.startDataGeneration(
            args.alpha,
            policy_file,
            time_step,
            data_decimation,
            samples,
            sampling_covariance,
            initial_observations,
            mode_schedules,
            target_trajectories,
        )
        while not interface.isDataGenerationDone():
            time.sleep(0.25)
        return list(interface.getGeneratedData())
    finally:
        if temp_dir is not None:
            temp_dir.cleanup()


def stack_data(data: List[object]) -> Dict[str, np.ndarray]:
    arrays = {
        "t": np.asarray([d.t for d in data], dtype=np.float64),
        "mode": np.asarray([d.mode for d in data], dtype=np.int64),
        "x": np.stack([np.asarray(d.x, dtype=np.float64) for d in data]),
        "u": np.stack([np.asarray(d.u, dtype=np.float64) for d in data]),
        "observation": np.stack([np.asarray(d.observation, dtype=np.float64) for d in data]),
        "action_matrix": np.stack([np.asarray(d.actionTransformation[0], dtype=np.float64) for d in data]),
        "action_vector": np.stack([np.asarray(d.actionTransformation[1], dtype=np.float64) for d in data]),
        "H": np.asarray([d.hamiltonian.f for d in data], dtype=np.float64),
        "dHdx": np.stack([np.asarray(d.hamiltonian.dfdx, dtype=np.float64) for d in data]),
        "dHdu": np.stack([np.asarray(d.hamiltonian.dfdu, dtype=np.float64) for d in data]),
        "dHdxx": np.stack([np.asarray(d.hamiltonian.dfdxx, dtype=np.float64) for d in data]),
        "dHdux": np.stack([np.asarray(d.hamiltonian.dfdux, dtype=np.float64) for d in data]),
        "dHduu": np.stack([np.asarray(d.hamiltonian.dfduu, dtype=np.float64) for d in data]),
    }
    arrays["x_norm"] = np.linalg.norm(arrays["x"], axis=1)
    arrays["u_norm"] = np.linalg.norm(arrays["u"], axis=1)
    arrays["observation_norm"] = np.linalg.norm(arrays["observation"], axis=1)
    arrays["dHdu_norm"] = np.linalg.norm(arrays["dHdu"], axis=1)
    sym_dHduu = 0.5 * (arrays["dHduu"] + np.swapaxes(arrays["dHduu"], 1, 2))
    arrays["dHduu_eigs"] = np.linalg.eigvalsh(sym_dHduu)
    arrays["dHduu_min_eig"] = arrays["dHduu_eigs"].min(axis=1)
    return arrays


def finite_values(values: np.ndarray) -> np.ndarray:
    flat = np.asarray(values, dtype=np.float64).reshape(-1)
    return flat[np.isfinite(flat)]


def summarize_array(values: np.ndarray) -> Dict[str, object]:
    arr = np.asarray(values)
    finite = finite_values(arr)
    result: Dict[str, object] = {
        "shape": list(arr.shape),
        "count": int(arr.size),
        "finite_count": int(finite.size),
        "nonfinite_count": int(arr.size - finite.size),
        "finite_ratio": float(finite.size / arr.size) if arr.size else 0.0,
    }
    if finite.size:
        result.update(
            {
                "min": float(np.min(finite)),
                "p01": float(np.percentile(finite, 1)),
                "mean": float(np.mean(finite)),
                "std": float(np.std(finite)),
                "p50": float(np.percentile(finite, 50)),
                "p99": float(np.percentile(finite, 99)),
                "max": float(np.max(finite)),
            }
        )
    return result


def build_summary(arrays: Dict[str, np.ndarray], args: argparse.Namespace) -> Dict[str, object]:
    checks = []
    if arrays["t"].size == 0:
        checks.append({"level": "FAIL", "message": "No data points were generated."})
    for name in ["x", "u", "observation", "H", "dHdx", "dHdu", "dHdxx", "dHdux", "dHduu"]:
        nonfinite = summarize_array(arrays[name])["nonfinite_count"]
        if nonfinite:
            checks.append({"level": "FAIL", "message": f"{name} has {nonfinite} non-finite values."})
    min_eig = finite_values(arrays["dHduu_min_eig"])
    if min_eig.size and np.min(min_eig) < -1.0e-6:
        checks.append(
            {
                "level": "WARN",
                "message": f"dHduu has negative eigenvalues; minimum is {np.min(min_eig):.6g}.",
            }
        )
    if not checks:
        checks.append({"level": "PASS", "message": "Basic finite-value checks passed."})

    return {
        "generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "arguments": vars(args),
        "data_points": int(arrays["t"].size),
        "time_min": float(np.min(arrays["t"])) if arrays["t"].size else None,
        "time_max": float(np.max(arrays["t"])) if arrays["t"].size else None,
        "mode_counts": {str(k): int(v) for k, v in zip(*np.unique(arrays["mode"], return_counts=True))},
        "checks": checks,
        "stats": {name: summarize_array(value) for name, value in arrays.items()},
    }


def nice_number(value: float) -> str:
    if not np.isfinite(value):
        return "nan"
    if abs(value) >= 1.0e4 or (abs(value) < 1.0e-3 and value != 0.0):
        return f"{value:.3e}"
    return f"{value:.4g}"


def svg_axes(width: int, height: int, title: str, subtitle: str = "") -> str:
    escaped_title = html.escape(title)
    escaped_subtitle = html.escape(subtitle)
    return (
        f"<svg viewBox='0 0 {width} {height}' width='{width}' height='{height}' role='img'>"
        f"<rect width='{width}' height='{height}' fill='#ffffff'/>"
        f"<text x='14' y='22' font-size='15' font-weight='700'>{escaped_title}</text>"
        f"<text x='14' y='40' font-size='11' fill='#666'>{escaped_subtitle}</text>"
    )


def svg_no_data(title: str, message: str) -> str:
    width, height = 360, 220
    return (
        svg_axes(width, height, title, message)
        + "<rect x='14' y='56' width='332' height='140' fill='#f7f7f7' stroke='#ddd'/>"
        + "<text x='180' y='130' text-anchor='middle' font-size='12' fill='#777'>no finite data</text>"
        + "</svg>"
    )


def svg_hist(values: np.ndarray, title: str, bins: int = 40) -> str:
    width, height = 360, 220
    margin_left, margin_right, margin_top, margin_bottom = 46, 14, 56, 34
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    finite = finite_values(values)
    if finite.size == 0:
        return svg_no_data(title, "histogram")
    if np.allclose(np.min(finite), np.max(finite)):
        finite = np.concatenate([finite, finite + 1.0e-12])
    hist_bins = max(5, min(bins, int(np.sqrt(finite.size)) + 1))
    counts, edges = np.histogram(finite, bins=hist_bins)
    max_count = max(int(np.max(counts)), 1)
    subtitle = f"n={finite.size}, min={nice_number(np.min(finite))}, max={nice_number(np.max(finite))}"
    parts = [svg_axes(width, height, title, subtitle)]
    parts.append(f"<line x1='{margin_left}' y1='{height - margin_bottom}' x2='{width - margin_right}' y2='{height - margin_bottom}' stroke='#999'/>")
    parts.append(f"<line x1='{margin_left}' y1='{margin_top}' x2='{margin_left}' y2='{height - margin_bottom}' stroke='#999'/>")
    bar_gap = 1.0
    for i, count in enumerate(counts):
        x = margin_left + i * plot_width / len(counts)
        bar_width = max(1.0, plot_width / len(counts) - bar_gap)
        bar_height = plot_height * count / max_count
        y = height - margin_bottom - bar_height
        parts.append(
            f"<rect x='{x:.2f}' y='{y:.2f}' width='{bar_width:.2f}' height='{bar_height:.2f}' fill='#2f6f9f'/>"
        )
    parts.append(f"<text x='{margin_left}' y='{height - 10}' font-size='10' fill='#666'>{nice_number(edges[0])}</text>")
    parts.append(f"<text x='{width - margin_right}' y='{height - 10}' text-anchor='end' font-size='10' fill='#666'>{nice_number(edges[-1])}</text>")
    parts.append(f"<text x='12' y='{margin_top + 8}' font-size='10' fill='#666'>{max_count}</text>")
    parts.append("</svg>")
    return "".join(parts)


def svg_series(values: np.ndarray, title: str, max_points: int = 800) -> str:
    width, height = 720, 240
    margin_left, margin_right, margin_top, margin_bottom = 54, 16, 56, 34
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    y = np.asarray(values, dtype=np.float64).reshape(-1)
    finite_mask = np.isfinite(y)
    if not finite_mask.any():
        return svg_no_data(title, "series")
    if y.size > max_points:
        idx = np.linspace(0, y.size - 1, max_points).astype(int)
        y = y[idx]
        finite_mask = finite_mask[idx]
    finite = y[finite_mask]
    ymin, ymax = float(np.min(finite)), float(np.max(finite))
    if np.isclose(ymin, ymax):
        ymin -= 1.0
        ymax += 1.0
    subtitle = f"n={values.size}, min={nice_number(np.min(finite))}, max={nice_number(np.max(finite))}"
    parts = [svg_axes(width, height, title, subtitle)]
    parts.append(f"<line x1='{margin_left}' y1='{height - margin_bottom}' x2='{width - margin_right}' y2='{height - margin_bottom}' stroke='#999'/>")
    parts.append(f"<line x1='{margin_left}' y1='{margin_top}' x2='{margin_left}' y2='{height - margin_bottom}' stroke='#999'/>")
    points = []
    denom = max(1, y.size - 1)
    for i, value in enumerate(y):
        if not np.isfinite(value):
            continue
        px = margin_left + plot_width * i / denom
        py = height - margin_bottom - plot_height * (value - ymin) / (ymax - ymin)
        points.append(f"{px:.2f},{py:.2f}")
    if points:
        parts.append(f"<polyline points='{' '.join(points)}' fill='none' stroke='#a23e48' stroke-width='1.4'/>")
    parts.append(f"<text x='10' y='{margin_top + 6}' font-size='10' fill='#666'>{nice_number(ymax)}</text>")
    parts.append(f"<text x='10' y='{height - margin_bottom}' font-size='10' fill='#666'>{nice_number(ymin)}</text>")
    parts.append("</svg>")
    return "".join(parts)


def stats_table(summary: Dict[str, object], names: Iterable[str]) -> str:
    rows = []
    for name in names:
        stats = summary["stats"][name]
        rows.append(
            "<tr>"
            f"<td>{html.escape(name)}</td>"
            f"<td>{html.escape(str(stats['shape']))}</td>"
            f"<td>{stats['finite_ratio']:.3f}</td>"
            f"<td>{nice_number(stats.get('min', np.nan))}</td>"
            f"<td>{nice_number(stats.get('mean', np.nan))}</td>"
            f"<td>{nice_number(stats.get('std', np.nan))}</td>"
            f"<td>{nice_number(stats.get('p99', np.nan))}</td>"
            f"<td>{nice_number(stats.get('max', np.nan))}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>array</th><th>shape</th><th>finite</th><th>min</th>"
        "<th>mean</th><th>std</th><th>p99</th><th>max</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def dimension_plots(name: str, values: np.ndarray, labels: List[str]) -> str:
    parts = [f"<h2>{html.escape(name)}</h2>", "<div class='grid'>"]
    for i in range(values.shape[1]):
        label = labels[i] if i < len(labels) else f"{name}_{i}"
        parts.append(svg_hist(values[:, i], label))
    parts.append("</div>")
    return "".join(parts)


def write_report(output_dir: Path, arrays: Dict[str, np.ndarray], summary: Dict[str, object]) -> Path:
    checks = "".join(
        f"<li class='{item['level'].lower()}'><strong>{html.escape(item['level'])}</strong> {html.escape(item['message'])}</li>"
        for item in summary["checks"]
    )
    body = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<title>Ballbot MPC-Net Data Quality</title>",
        "<style>",
        "body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;margin:24px;background:#f4f6f8;color:#202124}",
        "h1{margin-bottom:4px} h2{margin-top:28px}",
        ".card{background:white;border:1px solid #d8dde3;border-radius:8px;padding:16px;margin:14px 0}",
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:12px}",
        "svg{background:white;border:1px solid #dde2e6;border-radius:6px;max-width:100%;height:auto}",
        "table{border-collapse:collapse;width:100%;font-size:13px} th,td{border-bottom:1px solid #e4e7eb;padding:6px;text-align:right} th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}",
        ".pass{color:#186a3b}.warn{color:#9a6700}.fail{color:#b42318}",
        "code{background:#eef1f4;padding:1px 4px;border-radius:4px}",
        "</style></head><body>",
        "<h1>Ballbot MPC-Net Data Quality</h1>",
        f"<p>Generated at <code>{html.escape(summary['generated_at'])}</code>. Data points: <strong>{summary['data_points']}</strong>.</p>",
        "<div class='card'><h2>Checks</h2><ul>",
        checks,
        "</ul></div>",
        "<div class='card'><h2>Summary</h2>",
        stats_table(summary, ["x", "u", "observation", "H", "dHdu_norm", "dHduu_min_eig"]),
        "</div>",
        "<div class='card'><h2>Key Scalars</h2><div class='grid'>",
        svg_hist(arrays["u_norm"], "input norm"),
        svg_hist(arrays["observation_norm"], "observation norm"),
        svg_hist(arrays["H"], "Hamiltonian H"),
        svg_hist(arrays["dHdu_norm"], "Hamiltonian input gradient norm"),
        svg_hist(arrays["dHduu_min_eig"], "min eig(dHduu)"),
        svg_series(arrays["u_norm"], "input norm in generated order"),
        "</div></div>",
        "<div class='card'>",
        dimension_plots("Input u", arrays["u"], INPUT_LABELS),
        "</div>",
        "<div class='card'>",
        dimension_plots("State x", arrays["x"], STATE_LABELS),
        "</div>",
        "<div class='card'>",
        dimension_plots("Observation", arrays["observation"], STATE_LABELS),
        "</div>",
        "</body></html>",
    ]
    report_path = output_dir / "report.html"
    report_path.write_text("".join(body), encoding="utf-8")
    return report_path


def main() -> None:
    args = parse_args()
    package_dir = Path(__file__).resolve().parent
    output_dir = make_output_dir(package_dir, args.output_dir)

    config = Config(args.config)
    config.DEVICE = torch.device(args.device)

    print(f"[data_quality] output directory: {output_dir}", flush=True)
    print("[data_quality] generating MPC data...", flush=True)
    data = run_data_generation(config, args)
    if not data:
        raise RuntimeError("Data generation returned no samples. Check the preceding MPC rollout messages.")

    print(f"[data_quality] received {len(data)} data points", flush=True)
    arrays = stack_data(data)
    summary = build_summary(arrays, args)

    np.savez_compressed(output_dir / "raw_data.npz", **arrays)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
    report_path = write_report(output_dir, arrays, summary)

    print(f"[data_quality] wrote {report_path}", flush=True)
    print(f"[data_quality] wrote {output_dir / 'summary.json'}", flush=True)
    print(f"[data_quality] wrote {output_dir / 'raw_data.npz'}", flush=True)
    for check in summary["checks"]:
        print(f"[data_quality] {check['level']}: {check['message']}", flush=True)


if __name__ == "__main__":
    main()
