#!/usr/bin/python3 -B

# Copyright 2021 Josh Pieper, jjp@pobox.com.
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


"""
This example commands a single servo at ID #1 using the default
transport to hold the current position indefinitely, and prints the
state of the servo to the console.
"""

import asyncio
import math
import pickle
import moteus
import time

from numpy import maximum

data = []


async def main():
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    c = moteus.Controller()
    try:
        # In case the controller had faulted previously, at the start of
        # this script we send the stop command in order to clear it.
        await c.set_stop()

        dt_filtered = 0
        last_time = time.time()
        timestamp_begin_loop = time.time()

        last_position = 0.0

        torque_to_pd = TorqueToPD(c,
                                  t_begin=0.4,
                                  t_reset=0.8,
                                  pd_begin=2.0,
                                  pd_target=2.5,
                                  torque=1.5,
                                  maximum_torque=1.5,
                                  termination_time=1.2)

        while True:
            loop_start = time.time()
            time_since_start = loop_start - timestamp_begin_loop
            dt = loop_start - last_time
            dt_filtered = 0.99 * dt_filtered + 0.01 * dt
            last_time = loop_start

            # state = move_waypoints(c, time_since_start)
            state = await torque_to_pd(last_position, time_since_start)
            last_position = state.values[moteus.Register.POSITION]

            data.append((time_since_start, state))

            # Print out just the position register.
            print(
                f"Position: {state.values[moteus.Register.POSITION]:0.3f} dt: {dt:0.4f} dt_filtered: {dt_filtered:0.4f}")

            if time_since_start > torque_to_pd.termination_time:
                return

            # await asyncio.sleep(0.001)
            # without waiting the loop time is about 400Hz!
    finally:
        await c.set_stop()
        with open("log.pickle", "wb+") as f:
            pickle.dump(data, f)


class TorqueToPD:
    def __init__(self,
                 controller,
                 t_begin,
                 t_reset,
                 pd_begin,
                 pd_target,
                 torque,
                 kp_scale=1.0,
                 kd_scale=1.0,
                 maximum_torque=1.0,
                 termination_time=1.0,
                 reset_torque=0.2):
        self.controller = controller
        self.t_begin = t_begin
        self.t_reset = t_reset
        self.pd_begin = pd_begin
        self.pd_target = pd_target
        self.torque = torque
        self.kp_scale = kp_scale
        self.kd_scale = kd_scale
        self.maximum_torque = maximum_torque
        self.reset_torque = reset_torque

        self.past_pd_threshold = False

        self.termination_time = termination_time

    async def __call__(self, pos, t):
        if t < self.t_begin:
            return await self.controller.set_position(position=0.0,
                                                      kp_scale=self.kp_scale,
                                                      kd_scale=self.kd_scale,
                                                      maximum_torque=self.maximum_torque,
                                                      query=True)
        elif pos < self.pd_begin and not self.past_pd_threshold:
            return await self.controller.set_position(maximum_torque=self.maximum_torque,
                                                      kp_scale=0.0,
                                                      kd_scale=0.0,
                                                      feedforward_torque=self.torque,
                                                      query=True)
        elif t < self.t_reset:
            self.past_pd_threshold = True
            return await self.controller.set_position(position=self.pd_target,
                                                      kp_scale=self.kp_scale,
                                                      kd_scale=self.kd_scale,
                                                      maximum_torque=self.maximum_torque,
                                                      query=True
                                                      )
        else:
            return await self.controller.set_position(position=0,
                                                      kp_scale=self.kp_scale,
                                                      kd_scale=self.kd_scale,
                                                      maximum_torque=self.reset_torque,
                                                      query=True
                                                      )


async def move_waypoints(c,
                         t,
                         kp_scale=0.3,
                         kd_scale=0.008,
                         maximum_torque=1.0,
                         POS_0=0.0,
                         TIME_1=1.0,  # time to move to
                         POS_1=3.0,  # rots
                         TIME_2=2.0,
                         POS_END=0.0):
    pos_command = 0.0
    if t < TIME_1:
        pos_command = POS_0
    elif t < TIME_2:
        pos_command = POS_1
    else:
        pos_command = POS_END
    return await c.set_position(position=pos_command,
                                maximum_torque=maximum_torque,
                                kp_scale=kp_scale,
                                kd_scale=kd_scale)


if __name__ == '__main__':
    asyncio.run(main())
