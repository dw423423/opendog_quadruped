###############################################################################
# Copyright (c) 2022, Farbod Farshidian. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""MPC-Net class.

Provides a class that handles the MPC-Net training.
"""

import datetime
import json
import os
import shutil
import time
from abc import ABCMeta, abstractmethod
from typing import Optional, Tuple

import numpy as np
import torch
import torch.onnx
from ocs2_mpcnet_core.config import Config
from ocs2_mpcnet_core.loss import BaseLoss
from ocs2_mpcnet_core.memory import BaseMemory
from ocs2_mpcnet_core.policy import BasePolicy
from torch.utils.tensorboard import SummaryWriter

from ocs2_mpcnet_core import SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray
from ocs2_mpcnet_core import helper


class Mpcnet(metaclass=ABCMeta):
    """MPC-Net.

    Implements the main methods for the MPC-Net training.

    Takes a specific configuration, interface, memory, policy and loss function(s).
    The task formulation has to be implemented in a robot-specific class derived from this class.
    Provides the main training loop for MPC-Net.
    """

    def __init__(
            self,
            root_dir: str,
            config: Config,
            interface: object,
            memory: BaseMemory,
            policy: BasePolicy,
            experts_loss: BaseLoss,
            gating_loss: Optional[BaseLoss] = None,
            pt_file_path: Optional[str] = None,
    ) -> None:
        """Initializes the Mpcnet class.

        Initializes the Mpcnet class by setting fixed and variable attributes.

        Args:
            root_dir: The absolute path to the root directory.
            config: An instance of the configuration class.
            interface: An instance of the interface class.
            memory: An instance of a memory class.
            policy: An instance of a policy class.
            experts_loss: An instance of a loss class used as experts loss.
            gating_loss: An instance of a loss class used as gating loss.
        """
        # config
        self.config = config
        # interface
        self.interface = interface
        # logging
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(root_dir, "runs", f"{timestamp}_{config.NAME}_{config.DESCRIPTION}")
        self.writer = SummaryWriter(self.log_dir)
        # loss
        self.experts_loss = experts_loss
        self.gating_loss = gating_loss
        # memory
        self.memory = memory
        # policy
        if pt_file_path:
            self.policy = torch.load(pt_file_path, weights_only=False)
            print("==============\nLoaded policy from", pt_file_path, "\n==============")
        else:
            self.policy = policy
        self.policy.to(config.DEVICE)
        self.dummy_observation = torch.randn(1, config.OBSERVATION_DIM, device=config.DEVICE, dtype=config.DTYPE)
        # optimizer
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=config.LEARNING_RATE)

    def save_policy(self, policy: BasePolicy, save_path: str) -> None:
        """Save a policy as ONNX and PyTorch files without the extension."""
        torch.onnx.export(
            model=policy,
            args=self.dummy_observation,
            f=save_path + ".onnx",
            dynamo=False,
            opset_version=11,
        )
        torch.save(obj=policy, f=save_path + ".pt")

    def _log_mpc_fit_metrics(
            self,
            predicted_input: torch.Tensor,
            mpc_input: torch.Tensor,
            empirical_experts_loss: torch.Tensor,
            nominal_hamiltonian: torch.Tensor,
            iteration: int,
    ) -> None:
        """Log how closely the policy input matches the MPC input on a training batch."""
        log_interval = int(getattr(self.config, "MPC_FIT_LOG_INTERVAL", 0))
        if log_interval <= 0 or iteration % log_interval != 0:
            return

        with torch.no_grad():
            input_error = predicted_input.detach() - mpc_input
            input_rmse = torch.sqrt(torch.mean(input_error.square()))
            input_mae = torch.mean(torch.abs(input_error))
            hamiltonian_gap = empirical_experts_loss.detach() - torch.mean(nominal_hamiltonian)

            self.writer.add_scalar("fit/input_rmse", input_rmse.item(), iteration)
            self.writer.add_scalar("fit/input_mae", input_mae.item(), iteration)
            self.writer.add_scalar("fit/hamiltonian_gap", hamiltonian_gap.item(), iteration)

            action_scaling = getattr(self.config, "ACTION_SCALING", None)
            if action_scaling is not None and len(action_scaling) == input_error.shape[1]:
                input_scaling = torch.as_tensor(
                    action_scaling,
                    device=input_error.device,
                    dtype=input_error.dtype,
                )
                if torch.any(input_scaling == 0.0):
                    raise ValueError("ACTION_SCALING must be nonzero to log normalized MPC fit error.")
                normalized_input_rmse = torch.sqrt(torch.mean((input_error / input_scaling).square()))
                self.writer.add_scalar(
                    "fit/normalized_input_rmse", normalized_input_rmse.item(), iteration
                )

            input_groups = getattr(self.config, "MPC_FIT_INPUT_GROUPS", {})
            for group_name, bounds in input_groups.items():
                if len(bounds) != 2:
                    raise ValueError(
                        f"MPC_FIT_INPUT_GROUPS['{group_name}'] must contain [start, end] indices."
                    )
                start, end = int(bounds[0]), int(bounds[1])
                if not (0 <= start < end <= input_error.shape[1]):
                    raise ValueError(
                        f"MPC_FIT_INPUT_GROUPS['{group_name}']={bounds} is outside input dimension "
                        f"{input_error.shape[1]}."
                    )
                group_rmse = torch.sqrt(torch.mean(input_error[:, start:end].square()))
                self.writer.add_scalar(f"fit/{group_name}_rmse", group_rmse.item(), iteration)

    @staticmethod
    def is_better_policy(
            survival_time: float,
            final_xy_error: float,
            final_yaw_error: float,
            incurred_hamiltonian: float,
            evaluation_duration: float,
            best_survival_time: float,
            best_final_xy_error: float,
            best_final_yaw_error: float,
            best_incurred_hamiltonian: float,
    ) -> bool:
        """Return true if full survival is reached, then final xy error improves."""
        if not np.isfinite(survival_time):
            return False

        eps = 1e-9
        survival_complete = survival_time >= evaluation_duration - eps
        best_survival_complete = best_survival_time >= evaluation_duration - eps
        if survival_complete != best_survival_complete:
            return survival_complete

        if not survival_complete:
            if survival_time > best_survival_time + eps:
                return True
            if survival_time < best_survival_time - eps:
                return False

        current_xy_error = final_xy_error if np.isfinite(final_xy_error) else np.inf
        best_xy_error = best_final_xy_error if np.isfinite(best_final_xy_error) else np.inf
        if current_xy_error < best_xy_error - eps:
            return True
        if current_xy_error > best_xy_error + eps:
            return False

        current_yaw_error = final_yaw_error if np.isfinite(final_yaw_error) else np.inf
        best_yaw_error = best_final_yaw_error if np.isfinite(best_final_yaw_error) else np.inf
        if current_yaw_error < best_yaw_error - eps:
            return True
        if current_yaw_error > best_yaw_error + eps:
            return False

        current_hamiltonian = incurred_hamiltonian if np.isfinite(incurred_hamiltonian) else np.inf
        best_hamiltonian = best_incurred_hamiltonian if np.isfinite(best_incurred_hamiltonian) else np.inf
        return current_hamiltonian < best_hamiltonian - eps

    @abstractmethod
    def get_tasks(
            self, tasks_number: int, duration: float
    ) -> Tuple[SystemObservationArray, ModeScheduleArray, TargetTrajectoriesArray]:
        """Get tasks.

        Get a random set of task that should be executed by the data generation or policy evaluation.

        Args:
            tasks_number: Number of tasks given by an integer.
            duration: Duration of each task given by a float.

        Returns:
            A tuple containing the components of the task.
                - initial_observations: The initial observations given by an OCS2 system observation array.
                - mode_schedules: The desired mode schedules given by an OCS2 mode schedule array.
                - target_trajectories: The desired target trajectories given by an OCS2 target trajectories array.
        """
        pass

    def start_data_generation(self, policy: BasePolicy, alpha: float = 1.0):
        """Start data generation.

        Start the data generation rollouts to receive new data.

        Args:
            policy: The current learned policy.
            alpha: The weight of the MPC policy in the rollouts.
        """
        policy_file_path = "/tmp/data_generation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        torch.onnx.export(model=policy, args=self.dummy_observation, f=policy_file_path, dynamo=False, opset_version=11,)
        initial_observations, mode_schedules, target_trajectories = self.get_tasks(
            self.config.DATA_GENERATION_TASKS, self.config.DATA_GENERATION_DURATION
        )
        self.interface.startDataGeneration(
            alpha,
            policy_file_path,
            self.config.DATA_GENERATION_TIME_STEP,
            self.config.DATA_GENERATION_DATA_DECIMATION,
            self.config.DATA_GENERATION_SAMPLES,
            np.diag(np.power(np.array(self.config.DATA_GENERATION_SAMPLING_VARIANCE), 2)),
            initial_observations,
            mode_schedules,
            target_trajectories,
        )

    def start_policy_evaluation(self, policy: BasePolicy, alpha: float = 0.0):
        """Start policy evaluation.

        Start the policy evaluation rollouts to validate the current performance.

        Args:
            policy: The current learned policy.
            alpha: The weight of the MPC policy in the rollouts.
        """
        policy_file_path = "/tmp/policy_evaluation_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".onnx"
        pt_file_path = policy_file_path[:-5] + ".pt"
        torch.onnx.export(model=policy, args=self.dummy_observation, f=policy_file_path, dynamo=False, opset_version=11,)
        torch.save(obj=policy, f=pt_file_path)
        initial_observations, mode_schedules, target_trajectories = self.get_tasks(
            self.config.POLICY_EVALUATION_TASKS, self.config.POLICY_EVALUATION_DURATION
        )
        self.interface.startPolicyEvaluation(
            alpha,
            policy_file_path,
            self.config.POLICY_EVALUATION_TIME_STEP,
            initial_observations,
            mode_schedules,
            target_trajectories,
        )
        return policy_file_path, pt_file_path

    def train(self) -> None:
        """Train.

        Run the main training loop of MPC-Net.
        """
        try:
            best_survival_time = -np.inf
            best_final_xy_error = np.inf
            best_final_yaw_error = np.inf
            best_incurred_hamiltonian = np.inf
            best_policy_info_path = os.path.join(self.log_dir, "best_policy.json")

            # save initial policy
            save_path = self.log_dir + "/initial_policy"
            self.save_policy(self.policy, save_path)

            print("==============\nWaiting for first data.\n==============")
            self.start_data_generation(self.policy)
            pending_policy_evaluation_files = self.start_policy_evaluation(self.policy)
            while not self.interface.isDataGenerationDone():
                time.sleep(1.0)

            print("==============\nStarting training.\n==============")
            for iteration in range(self.config.LEARNING_ITERATIONS):
                alpha = 1.0 - 1.0 * iteration / self.config.LEARNING_ITERATIONS

                # data generation
                if self.interface.isDataGenerationDone():
                    # get generated data
                    data = self.interface.getGeneratedData()
                    for i in range(len(data)):
                        # push t, x, u, p, observation, action transformation, Hamiltonian into memory
                        self.memory.push(
                            data[i].t,
                            data[i].x,
                            data[i].u,
                            helper.get_one_hot(data[i].mode, self.config.EXPERT_NUM, self.config.EXPERT_FOR_MODE),
                            data[i].observation,
                            data[i].actionTransformation,
                            data[i].hamiltonian,
                        )
                    # logging
                    self.writer.add_scalar("data/new_data_points", len(data), iteration)
                    self.writer.add_scalar("data/total_data_points", len(self.memory), iteration)
                    print("iteration", iteration, "received data points", len(data), "requesting with alpha", alpha)
                    # start new data generation
                    self.start_data_generation(self.policy, alpha)

                # policy evaluation
                if self.interface.isPolicyEvaluationDone():
                    # get computed metrics
                    metrics = self.interface.getComputedMetrics()
                    survival_time = np.mean([metrics[i].survivalTime for i in range(len(metrics))])
                    final_xy_error = np.mean([getattr(metrics[i], "finalXyError", np.inf) for i in range(len(metrics))])
                    final_yaw_error = np.mean([getattr(metrics[i], "finalYawError", np.inf) for i in range(len(metrics))])
                    incurred_hamiltonian = np.mean([metrics[i].incurredHamiltonian for i in range(len(metrics))])
                    # logging
                    self.writer.add_scalar("metric/survival_time", survival_time, iteration)
                    self.writer.add_scalar("metric/final_xy_error", final_xy_error, iteration)
                    self.writer.add_scalar("metric/final_yaw_error", final_yaw_error, iteration)
                    self.writer.add_scalar("metric/incurred_hamiltonian", incurred_hamiltonian, iteration)
                    if self.is_better_policy(
                            survival_time, final_xy_error, final_yaw_error, incurred_hamiltonian,
                            self.config.POLICY_EVALUATION_DURATION, best_survival_time, best_final_xy_error,
                            best_final_yaw_error, best_incurred_hamiltonian):
                        best_survival_time = survival_time
                        best_final_xy_error = final_xy_error
                        best_final_yaw_error = final_yaw_error
                        best_incurred_hamiltonian = incurred_hamiltonian
                        shutil.copyfile(pending_policy_evaluation_files[0], self.log_dir + "/best_policy.onnx")
                        shutil.copyfile(pending_policy_evaluation_files[1], self.log_dir + "/best_policy.pt")
                        with open(best_policy_info_path, "w") as stream:
                            json.dump(
                                {
                                    "iteration": iteration,
                                    "evaluation_duration": float(self.config.POLICY_EVALUATION_DURATION),
                                    "survival_time": float(survival_time),
                                    "final_xy_error": (
                                        float(final_xy_error) if np.isfinite(final_xy_error) else None
                                    ),
                                    "final_yaw_error": (
                                        float(final_yaw_error) if np.isfinite(final_yaw_error) else None
                                    ),
                                    "incurred_hamiltonian": (
                                        float(incurred_hamiltonian) if np.isfinite(incurred_hamiltonian) else None
                                    ),
                                    "selection_priority": [
                                        "survival_time reaches evaluation_duration",
                                        "among complete evaluations, final_xy_error smaller",
                                        "final_yaw_error smaller only as a tie-breaker",
                                        "incurred_hamiltonian smaller only as a tie-breaker",
                                        "non-finite tie-breaker values are worse",
                                    ],
                                    "source_onnx": pending_policy_evaluation_files[0],
                                    "source_pt": pending_policy_evaluation_files[1],
                                },
                                stream,
                                indent=2,
                            )
                        print(
                            "iteration",
                            iteration,
                            "saved best policy:",
                            "survival_time",
                            best_survival_time,
                            "final_xy_error",
                            best_final_xy_error,
                            "final_yaw_error",
                            best_final_yaw_error,
                            "incurred_hamiltonian",
                            best_incurred_hamiltonian,
                        )
                    print(
                        "iteration",
                        iteration,
                        "received metrics:",
                        "final_xy_error",
                        final_xy_error,
                        "final_yaw_error",
                        final_yaw_error,
                        "incurred_hamiltonian",
                        incurred_hamiltonian,
                        "survival_time",
                        survival_time,
                    )
                    # start new policy evaluation
                    pending_policy_evaluation_files = self.start_policy_evaluation(self.policy)

                # save intermediate policy
                if (iteration % int(0.1 * self.config.LEARNING_ITERATIONS) == 0) and (iteration > 0):
                    save_path = self.log_dir + "/intermediate_policy_" + str(iteration)
                    self.save_policy(self.policy, save_path)

                # extract batch from memory
                (
                    t,
                    x,
                    u,
                    p,
                    observation,
                    action_transformation_matrix,
                    action_transformation_vector,
                    dHdxx,
                    dHdux,
                    dHduu,
                    dHdx,
                    dHdu,
                    H,
                ) = self.memory.sample(self.config.BATCH_SIZE)

                # normal closure only evaluating the experts loss
                def normal_closure():
                    # clear the gradients
                    self.optimizer.zero_grad()
                    # prediction
                    action = self.policy(observation)[0]
                    input = helper.bmv(action_transformation_matrix, action) + action_transformation_vector
                    # compute the empirical loss
                    empirical_loss = self.experts_loss(x, x, input, u, p, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    self._log_mpc_fit_metrics(input, u, empirical_loss, H, iteration)
                    # compute the gradients
                    empirical_loss.backward()
                    # clip the gradients
                    if self.config.GRADIENT_CLIPPING:
                        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.config.GRADIENT_CLIPPING_VALUE)
                    # logging
                    self.writer.add_scalar("objective/empirical_loss", empirical_loss.item(), iteration)
                    # return empirical loss
                    return empirical_loss

                # cheating closure also adding the gating loss (only relevant for mixture of experts networks)
                def cheating_closure():
                    # clear the gradients
                    self.optimizer.zero_grad()
                    # prediction
                    action, weights = self.policy(observation)[:2]
                    input = helper.bmv(action_transformation_matrix, action) + action_transformation_vector
                    # compute the empirical loss
                    empirical_experts_loss = self.experts_loss(x, x, input, u, p, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    empirical_gating_loss = self.gating_loss(x, x, u, u, weights, p, dHdxx, dHdux, dHduu, dHdx, dHdu, H)
                    empirical_loss = empirical_experts_loss + self.config.LAMBDA * empirical_gating_loss
                    self._log_mpc_fit_metrics(input, u, empirical_experts_loss, H, iteration)
                    # compute the gradients
                    empirical_loss.backward()
                    # clip the gradients
                    if self.config.GRADIENT_CLIPPING:
                        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.config.GRADIENT_CLIPPING_VALUE)
                    # logging
                    self.writer.add_scalar("objective/empirical_experts_loss", empirical_experts_loss.item(), iteration)
                    self.writer.add_scalar("objective/empirical_gating_loss", empirical_gating_loss.item(), iteration)
                    self.writer.add_scalar("objective/empirical_loss", empirical_loss.item(), iteration)
                    # return empirical loss
                    return empirical_loss

                # take an optimization step
                if self.config.CHEATING:
                    self.optimizer.step(cheating_closure)
                else:
                    self.optimizer.step(normal_closure)

                # let data generation and policy evaluation finish in last iteration (to avoid a segmentation fault)
                if iteration == self.config.LEARNING_ITERATIONS - 1:
                    while (not self.interface.isDataGenerationDone()) or (not self.interface.isPolicyEvaluationDone()):
                        time.sleep(1.0)

            print("==============\nTraining completed.\n==============")

            # save final policy
            save_path = self.log_dir + "/final_policy"
            self.save_policy(self.policy, save_path)

        except KeyboardInterrupt:
            # let data generation and policy evaluation finish (to avoid a segmentation fault)
            while (not self.interface.isDataGenerationDone()) or (not self.interface.isPolicyEvaluationDone()):
                time.sleep(1.0)
            print("==============\nTraining interrupted.\n==============")
            pass

        self.writer.close()
