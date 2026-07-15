#!/usr/bin/env python3

"""Compare a saved PyTorch MPC-Net policy with its ONNX export.

Usage:
    python compare_onnx.py path/to/best_policy.pt path/to/best_policy.onnx

The two files must originate from the same training run.  The script checks
near-equilibrium observations as well as reproducible random observations and
reports the per-action export error.
"""

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pt_path", type=Path, help="Saved PyTorch policy (.pt).")
    parser.add_argument("onnx_path", type=Path, help="Exported ONNX policy (.onnx).")
    parser.add_argument("--samples", type=int, default=1000, help="Number of random observations (default: 1000).")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for the random observations.")
    return parser.parse_args()


def make_observations(observation_dim: int, samples: int, seed: int) -> np.ndarray:
    if observation_dim != 10:
        raise ValueError(f"This Ballbot diagnostic expects OBSERVATION_DIM=10, received {observation_dim}.")
    if samples < 0:
        raise ValueError("--samples must be non-negative.")

    # Explicitly test equilibrium and the small-error region where steady-state
    # biases are most visible, followed by a reproducible broad test set.
    diagnostic_observations = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.01, -0.01, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.1, -0.1, 0.05, 0.02, -0.02, 0.1, -0.1, 0.05, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    random_observations = np.random.default_rng(seed).uniform(
        -1.0, 1.0, size=(samples, observation_dim)
    ).astype(np.float32)
    return np.vstack((diagnostic_observations, random_observations))


def main() -> None:
    args = parse_args()
    if not args.pt_path.is_file():
        raise FileNotFoundError(f"PyTorch policy not found: {args.pt_path}")
    if not args.onnx_path.is_file():
        raise FileNotFoundError(f"ONNX policy not found: {args.onnx_path}")

    torch_model = torch.load(args.pt_path, map_location="cpu", weights_only=False)
    torch_model.eval().cpu()
    observation_dim = torch_model.observation_dimension
    observations = make_observations(observation_dim, args.samples, args.seed)

    with torch.no_grad():
        # MPC-Net policies return a tuple; the first item is the action.
        torch_actions = torch_model(torch.from_numpy(observations))[0].cpu().numpy()

    session = ort.InferenceSession(str(args.onnx_path), providers=["CPUExecutionProvider"])
    input_info = session.get_inputs()[0]
    expected_batch = input_info.shape[0]
    if isinstance(expected_batch, int) and expected_batch == 1:
        # Policies are exported using a dummy (1, O) observation in mpcnet.py,
        # so this repository's ONNX files generally have a static batch size.
        onnx_actions = np.vstack(
            [session.run(None, {input_info.name: observation[None, :]})[0] for observation in observations]
        )
    else:
        onnx_actions = session.run(None, {input_info.name: observations})[0]

    if torch_actions.shape != onnx_actions.shape:
        raise RuntimeError(f"Output-shape mismatch: PyTorch {torch_actions.shape}, ONNX {onnx_actions.shape}.")

    error = onnx_actions - torch_actions
    np.set_printoptions(precision=10, suppress=False)
    print(f"observations compared: {len(observations)}")
    print("max abs difference:    ", np.max(np.abs(error)))
    print("mean abs difference:   ", np.mean(np.abs(error), axis=0))
    print("mean signed difference:", np.mean(error, axis=0))
    print("\nzero observation action:")
    print("PyTorch:", torch_actions[0])
    print("ONNX:   ", onnx_actions[0])
    print("diff:   ", error[0])


if __name__ == "__main__":
    main()
