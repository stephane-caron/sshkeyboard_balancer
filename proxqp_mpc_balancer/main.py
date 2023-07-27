#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control with the ProxQP solver."""

import argparse
import asyncio
import os
import time
from time import perf_counter
from typing import Optional

import gin
import gymnasium as gym
import mpacklog
import numpy as np
import proxsuite
import qpsolvers
import upkie.envs
from ltv_mpc import MPCQP, Plan, solve_mpc
from ltv_mpc.systems import CartPole
from qpsolvers import solve_problem
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.filters import low_pass_filter
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

upkie.envs.register()


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live-plot",
        help="Display a live plot of MPC trajectories",
        action="store_true",
        default=False,
    )
    return parser.parse_args()


@gin.configurable
class ProxQPWorkspace:
    def __init__(
        self, mpc_qp: MPCQP, update_preconditioner: bool, verbose: bool
    ):
        n_eq = 0
        n_in = mpc_qp.h.size // 2  # CartPole structure
        n = mpc_qp.P.shape[1]
        solver = proxsuite.proxqp.dense.QP(
            n,
            n_eq,
            n_in,
            dense_backend=proxsuite.proxqp.dense.DenseBackend.PrimalDualLDLT,
        )
        solver.settings.eps_abs = 1e-3
        solver.settings.eps_rel = 0.0
        solver.settings.verbose = verbose
        solver.settings.compute_timings = True
        solver.settings.primal_infeasibility_solving = True
        solver.init(
            H=mpc_qp.P,
            g=mpc_qp.q,
            C=mpc_qp.G[::2, :],  # CartPole structure
            l=-mpc_qp.h[1::2],  # CartPole structure
            u=mpc_qp.h[::2],  # CartPole structure
        )
        solver.solve()
        self.update_preconditioner = update_preconditioner
        self.solver = solver

    def solve(self, mpc_qp: MPCQP) -> qpsolvers.Solution:
        self.solver.update(
            g=mpc_qp.q,
            update_preconditioner=self.update_preconditioner,
        )
        self.solver.solve()
        qpsol = qpsolvers.Solution(mpc_qp.problem)
        qpsol.found = True
        qpsol.x = self.solver.results.x
        return qpsol


def get_target_states(
    cart_pole: CartPole, state: np.ndarray, target_vel: float
):
    """Define the reference state trajectory over the receding horizon.

    Args:
        state: Cart-pole state at the beginning of the horizon.
        target_vel: Target ground velocity in m/s.

    Returns:
        Goal state at the end of the horizon.
    """
    nx = CartPole.STATE_DIM
    T = cart_pole.sampling_period
    target_states = np.zeros((cart_pole.nb_timesteps + 1) * nx)
    for k in range(cart_pole.nb_timesteps + 1):
        target_states[k * nx] = state[0] + (k * T) * target_vel
        target_states[k * nx + 2] = target_vel
    return target_states


@gin.configurable
class UpkieCartPole(CartPole):
    def __init__(
        self,
        length: float,
        max_ground_accel: float,
        nb_timesteps: int,
        sampling_period: float,
    ):
        super().__init__(
            length=length,
            max_ground_accel=max_ground_accel,
            nb_timesteps=nb_timesteps,
            sampling_period=sampling_period,
        )


@gin.configurable
async def balance(
    env: gym.Env,
    logger: mpacklog.AsyncLogger,
    nb_env_steps: int,
    rebuild_qp_every_time: bool,
    show_live_plot: bool,
    stage_input_cost_weight: float,
    stage_state_cost_weight: float,
    terminal_cost_weight: float,
    warm_start: bool,
):
    """!
    Run proportional balancer in gym environment with logging.

    @param env Gym environment to Upkie.
    @param logger Additional logger.
    """
    cart_pole = UpkieCartPole()
    mpc_problem = cart_pole.build_mpc_problem(
        terminal_cost_weight=terminal_cost_weight,
        stage_state_cost_weight=stage_state_cost_weight,
        stage_input_cost_weight=stage_input_cost_weight,
    )
    mpc_problem.initial_state = np.zeros(4)
    mpc_qp = MPCQP(mpc_problem)
    proxqp = ProxQPWorkspace(mpc_qp)

    live_plot = None
    if show_live_plot and not on_raspi():
        from ltv_mpc.live_plots import CartPolePlot  # imports matplotlib

        live_plot = CartPolePlot(cart_pole, order="velocities")

    env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    commanded_velocity = 0.0
    planning_times = np.empty((nb_env_steps,)) if nb_env_steps > 0 else None
    step = 0
    while True:
        action[0] = commanded_velocity
        observation, _, terminated, truncated, info = await env.async_step(
            action
        )
        if terminated or truncated:
            observation, info = env.reset()

        observation_dict = info["observation"]
        ground_contact = observation_dict["floor_contact"]["contact"]

        # Unpack observation into initial MPC state
        (
            base_pitch,
            ground_position,
            base_angular_velocity,
            ground_velocity,
        ) = observation
        initial_state = np.array(
            [
                ground_position,
                base_pitch,
                ground_velocity,
                base_angular_velocity,
            ]
        )
        target_vel = 0.0
        target_states = get_target_states(cart_pole, initial_state, target_vel)

        mpc_problem.update_initial_state(initial_state)
        mpc_problem.update_goal_state(target_states[-CartPole.STATE_DIM :])
        mpc_problem.update_target_states(target_states[: -CartPole.STATE_DIM])

        t0 = perf_counter()
        if rebuild_qp_every_time:
            plan = solve_mpc(mpc_problem, solver="proxqp")
        else:
            mpc_qp.update_cost_vector(mpc_problem)
            if warm_start:
                qpsol = proxqp.solve(mpc_qp)
            else:
                qpsol = solve_problem(mpc_qp.problem, solver="proxqp")
            plan = Plan(mpc_problem, qpsol)
        if nb_env_steps > 0:
            planning_times[step] = perf_counter() - t0

        if not ground_contact:
            commanded_velocity = low_pass_filter(
                prev_output=commanded_velocity,
                cutoff_period=0.1,
                new_input=0.0,
                dt=env.dt,
            )
        elif plan.is_empty:
            logging.error("Solver found no solution to the MPC problem")
            logging.info("Continuing with previous action")
        else:  # plan was found
            cart_pole.state = initial_state
            if live_plot is not None:
                t = time.time()
                live_plot.update(plan, t, initial_state, t)
            commanded_accel = plan.first_input
            commanded_velocity = clamp_and_warn(
                commanded_velocity + commanded_accel * env.dt / 2.0,
                lower=-1.0,
                upper=+1.0,
                label="commanded_velocity",
            )
        await logger.put(  # log info to be written to file later
            {
                "action": action,
                "observation": info["observation"],
                "time": time.time(),
            }
        )

        if nb_env_steps > 0:
            step += 1
            if step >= nb_env_steps:
                break

    await logger.stop()
    report(mpc_problem, mpc_qp, planning_times)


def report(mpc_problem, mpc_qp, planning_times: Optional[np.ndarray]):
    average_ms = 1e3 * np.average(planning_times)
    std_ms = 1e3 * np.std(planning_times)
    nb_env_steps = planning_times.size
    print("")
    print(f"{gin.operative_config_str()}")
    print("")
    print(f"{mpc_problem.goal_state=}")
    print(f"{mpc_problem.nb_timesteps=}")
    print(f"{mpc_qp.P.shape=}")
    print(f"{mpc_qp.q.shape=}")
    print(f"{mpc_qp.G.shape=}")
    print(f"{mpc_qp.h.shape=}")
    print(f"{mpc_qp.Phi.shape=}")
    print(f"{mpc_qp.Psi.shape=}")
    print("")
    if planning_times is not None:
        print(
            "Planning time: "
            f"{average_ms:.2} ± {std_ms:.2} ms over {nb_env_steps} calls"
        )
        print("")


async def main(args):
    """Main function of our asyncio program."""
    logger = mpacklog.AsyncLogger("mpc_balancing.mpack")
    with gym.make("UpkieWheelsEnv-v4", frequency=200.0) as env:
        await asyncio.gather(
            balance(env, logger, show_live_plot=args.live_plot),
            logger.write(),  # write logs to file when there is time
        )


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()
    agent_dir = os.path.dirname(__file__)
    gin.parse_config_file(f"{agent_dir}/config.gin")
    args = parse_command_line_arguments()
    asyncio.run(main(args))
