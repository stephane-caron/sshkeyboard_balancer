#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import asyncio
import time

import gymnasium as gym
import mpacklog
import numpy as np
import proxsuite
import qpsolvers
import upkie.envs
from qpsolvers import solve_problem
from upkie.utils.clamp import clamp_and_warn
from upkie.utils.filters import low_pass_filter
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

from ltv_mpc import MPCQP, Plan
from ltv_mpc.systems import CartPole

upkie.envs.register()


def prepare_proxqp(mpc_qp: MPCQP, verbose: bool = False):
    n_eq = 0
    n_in = mpc_qp.h.size // 2  # CartPole structure
    n = mpc_qp.P.shape[1]
    proxqp = proxsuite.proxqp.dense.QP(
        n,
        n_eq,
        n_in,
        dense_backend=proxsuite.proxqp.dense.DenseBackend.PrimalDualLDLT,
    )
    proxqp.settings.eps_abs = 1e-3
    proxqp.settings.eps_rel = 0.0
    proxqp.settings.verbose = verbose
    proxqp.settings.compute_timings = True
    proxqp.settings.primal_infeasibility_solving = True
    proxqp.init(
        H=mpc_qp.P,
        g=mpc_qp.q,
        C=mpc_qp.G[::2, :],  # CartPole structure
        l=-mpc_qp.h[1::2],  # CartPole structure
        u=mpc_qp.h[::2],  # CartPole structure
    )
    proxqp.solve()
    return proxqp


def update_proxqp(proxqp, mpc_qp: MPCQP):
    proxqp.update(
        g=mpc_qp.q,
        update_preconditioner=True,  # TODO(scaron): test
    )
    proxqp.solve()
    qpsol = qpsolvers.Solution(mpc_qp.problem)
    qpsol.found = True
    qpsol.x = proxqp.results.x
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


async def balance(env: gym.Env, logger: mpacklog.AsyncLogger):
    """!
    Run proportional balancer in gym environment with logging.

    @param env Gym environment to Upkie.
    @param logger Additional logger.
    """
    cart_pole = CartPole(
        length=0.4,
        max_ground_accel=10.0,
        nb_timesteps=102,
        sampling_period=0.005,
    )
    mpc_problem = cart_pole.build_mpc_problem(
        terminal_cost_weight=10.0,
        stage_state_cost_weight=1.0,
        stage_input_cost_weight=1e-3,
    )
    mpc_problem.initial_state = np.zeros(4)
    mpc_qp = MPCQP(mpc_problem)
    proxqp = prepare_proxqp(mpc_qp)

    live_plot = None
    if not on_raspi():
        from ltv_mpc.live_plots import CartPolePlot  # imports matplotlib

        live_plot = CartPolePlot(cart_pole, order="velocities")

    env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    commanded_velocity = 0.0
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
        mpc_qp.update_cost_vector(mpc_problem)
        update_proxqp(proxqp, mpc_qp)

        qpsol = solve_problem(mpc_qp.problem, solver="proxqp")
        plan = Plan(mpc_problem, qpsol)
        if not ground_contact:
            logging.info("Waiting for ground contact")
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
                commanded_velocity + commanded_accel * env.dt,
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
    await logger.stop()


async def main():
    """Main function of our asyncio program."""
    logger = mpacklog.AsyncLogger("mpc_balancing.mpack")
    with gym.make("UpkieWheelsEnv-v4", frequency=200.0) as env:
        await asyncio.gather(
            balance(env, logger),
            logger.write(),  # write logs to file when there is time
        )


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()
    asyncio.run(main())
