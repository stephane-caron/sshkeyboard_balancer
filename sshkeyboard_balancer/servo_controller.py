#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any, Dict

import gin
import numpy as np
from upkie.utils.clamp import clamp

from wheel_balancer import WheelBalancer


@gin.configurable
class ServoController:
    """Balance Upkie using its wheels.

    Attributes:
        gain_scale: PD gain scale for hip and knee joints.
        position_right_in_left: Translation from the left contact frame to
            the right contact frame, expressed in the left contact frame.
        turning_gain_scale: Additional gain scale added when the robot is
            turning to keep the legs stiff while the ground pulls them apart.
    """

    gain_scale: float
    turning_gain_scale: float

    def __init__(
        self,
        gain_scale: float,
        turning_gain_scale: float,
        wheel_distance: float,
    ):
        """Create controller.

        Args:
            gain_scale: PD gain scale for hip and knee joints.
            turning_gain_scale: Additional gain scale added when the robot is
                turning to keep the legs stiff in spite of the ground pulling
                them apart.
            wheel_distance: Lateral distance between the two wheels in meters.
                This controller does not handle the case where the two wheels
                are not in the lateral plane.
        """
        self.alpha = 1.5
        self.gain_scale = clamp(gain_scale, 0.1, 2.0)
        self.position_right_in_left = np.array([0.0, wheel_distance, 0.0])
        self.servo_action = None
        self.turning_gain_scale = turning_gain_scale
        self.wheel_balancer = WheelBalancer()  # type: ignore

    def initialize_servo_action(self, observation: Dict[str, Any]) -> None:
        """Initialize default servo action from initial observation.

        Args:
            observation: Initial observation.
        """
        self.servo_action = {
            joint: {
                "position": observation["servo"][joint]["position"],
                "velocity": 0.0,
            }
            for joint in (
                f"{side}_{func}"
                for side in ("left", "right")
                for func in ("hip", "knee")
            )
        }
        self.servo_action.update(
            {
                wheel: {
                    "position": np.nan,
                    "velocity": 0.0,
                }
                for wheel in ("left_wheel", "right_wheel")
            }
        )

    def handle_inputs(self, observation: Dict[str, Any], dt: float):
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = 0.2 * axis_value
        except KeyError:
            velocity = 0.0
        dq = velocity * dt

        self.servo_action["left_hip"]["position"] += dq
        self.servo_action["left_knee"]["position"] += -self.alpha * dq
        self.servo_action["right_hip"]["position"] += -dq
        self.servo_action["right_knee"]["position"] += self.alpha * dq

        self.servo_action["left_hip"]["velocity"] = velocity
        self.servo_action["left_knee"]["velocity"] = -self.alpha * velocity
        self.servo_action["right_hip"]["velocity"] = -velocity
        self.servo_action["right_knee"]["velocity"] = self.alpha * velocity

        if "keyboard" not in observation:
            return
        key = observation["keyboard"].get("key", "")

        if key in ("left", "right"):
            alpha_incr = (+1.0 if key == "left" else -1.0) * 0.1
            self.alpha += alpha_incr
            print(f"Now at {self.alpha=}")
        assert 0.5 <= self.alpha <= 3.5

        if key in ("up", "down"):
            dq = (+1.0 if key == "up" else -1.0) * 0.02
            print(f"adding {dq=} rad")
            self.servo_action["left_hip"]["position"] += dq
            self.servo_action["right_hip"]["position"] -= dq
            self.servo_action["left_knee"]["position"] -= self.alpha * dq
            self.servo_action["right_knee"]["position"] += self.alpha * dq

    def cycle(self, observation: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if self.servo_action is None:
            self.initialize_servo_action(observation)

        self.handle_inputs(observation, dt)

        # Compute wheel velocities for balancing
        self.wheel_balancer.cycle(observation, dt)
        wheel_velocities = self.wheel_balancer.get_wheel_velocities(
            self.position_right_in_left
        )
        left_wheel_velocity, right_wheel_velocity = wheel_velocities
        self.servo_action["left_wheel"]["velocity"] = left_wheel_velocity
        self.servo_action["right_wheel"]["velocity"] = right_wheel_velocity

        # Increase leg stiffness while turning
        turning_prob = self.wheel_balancer.turning_probability
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            self.servo_action[joint_name]["kp_scale"] = kp_scale
            self.servo_action[joint_name]["kd_scale"] = kd_scale

        return {
            "servo": self.servo_action,
            "wheel_balancer": self.wheel_balancer.log(),
        }
