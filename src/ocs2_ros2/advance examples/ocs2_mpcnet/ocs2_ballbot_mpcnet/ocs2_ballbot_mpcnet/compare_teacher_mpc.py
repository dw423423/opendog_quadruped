#!/usr/bin/env python3

"""Measure steady-state action error between an ONNX policy and teacher MPC data.

First generate teacher data with ``data_quality.py --alpha 1.0``.  This script
loads its ``raw_data.npz`` file, selects near-equilibrium observations, and
compares the stored optimal MPC input ``u`` against the ONNX policy output.
"""

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_data", type=Path, help="raw_data.npz written by data_quality.py with --alpha 1.0.")
    parser.add_argument("onnx_path", type=Path, help="ONNX policy to compare with teacher MPC.")
    parser.add_argument("--xy-error-max", type=float, default=0.05, help="Maximum body-frame XY tracking error in m.")
    parser.add_argument("--yaw-error-max", type=float, default=0.05, help="Maximum yaw error in rad.")
    parser.add_argument("--attitude-error-max", type=float, default=0.10, help="Maximum pitch/roll error in rad.")
    parser.add_argument("--linear-speed-max", type=float, default=0.10, help="Maximum XY speed error in m/s.")
    parser.add_argument("--angular-speed-max", type=float, default=0.10, help="Maximum angular-speed error in rad/s.")
    return parser.parse_args()


def run_onnx(session: ort.InferenceSession, observations: np.ndarray) -> np.ndarray:
    input_info = session.get_inputs()[0]
    expected_batch = input_info.shape[0]
    if isinstance(expected_batch, int) and expected_batch == 1:
        return np.vstack([session.run(None, {input_info.name: obs[None, :]})[0] for obs in observations])
    return session.run(None, {input_info.name: observations})[0]


def steady_mask(observation: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    if observation.ndim != 2 or observation.shape[1] != 10:
        raise ValueError(f"Expected observation shape (N, 10), received {observation.shape}.")
    return (
        (np.linalg.norm(observation[:, 0:2], axis=1) <= args.xy_error_max)
        & (np.abs(observation[:, 2]) <= args.yaw_error_max)
        & (np.max(np.abs(observation[:, 3:5]), axis=1) <= args.attitude_error_max)
        & (np.linalg.norm(observation[:, 5:7], axis=1) <= args.linear_speed_max)
        & (np.max(np.abs(observation[:, 7:10]), axis=1) <= args.angular_speed_max)
    )


def main() -> None:
    args = parse_args()
    if not args.raw_data.is_file():
        raise FileNotFoundError(f"Teacher data not found: {args.raw_data}")
    if not args.onnx_path.is_file():
        raise FileNotFoundError(f"ONNX policy not found: {args.onnx_path}")

    with np.load(args.raw_data) as data:
        if "observation" not in data or "u" not in data:
            raise KeyError("raw_data.npz must contain 'observation' and teacher MPC input 'u'.")
        observations = np.asarray(data["observation"], dtype=np.float32)
        teacher_actions = np.asarray(data["u"], dtype=np.float32)

    finite = np.isfinite(observations).all(axis=1) & np.isfinite(teacher_actions).all(axis=1)
    selected = finite & steady_mask(observations, args)
    if not np.any(selected):
        raise RuntimeError("No near-equilibrium samples matched the thresholds. Increase rollout duration or relax --*-max.")

    observations = observations[selected]
    teacher_actions = teacher_actions[selected]
    session = ort.InferenceSession(str(args.onnx_path), providers=["CPUExecutionProvider"])
    network_actions = run_onnx(session, observations)
    if network_actions.shape != teacher_actions.shape:
        raise RuntimeError(f"Output-shape mismatch: ONNX {network_actions.shape}, MPC {teacher_actions.shape}.")

    difference = network_actions - teacher_actions
    nearest_index = int(np.argmin(np.linalg.norm(observations, axis=1)))
    np.set_printoptions(precision=8, suppress=False)
    print(f"steady samples: {len(observations)} / {len(selected)}")
    print("bias (u_nn - u_mpc):", np.mean(difference, axis=0))
    print("MAE:                ", np.mean(np.abs(difference), axis=0))
    print("max abs error:      ", np.max(np.abs(difference), axis=0))
    print("\nnearest sampled equilibrium point:")
    print("observation: ", observations[nearest_index])
    print("u_mpc:       ", teacher_actions[nearest_index])
    print("u_nn:        ", network_actions[nearest_index])
    print("u_nn-u_mpc:  ", difference[nearest_index])


if __name__ == "__main__":
    main()
