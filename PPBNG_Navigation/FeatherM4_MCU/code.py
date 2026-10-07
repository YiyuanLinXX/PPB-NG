# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# This file is derived from farm-ng Feather M4 CAN example code.

# Modified by Yiyuan Lin (yl3663@cornell.edu)
# Changes include ROS serial command parsing, Feather-side watchdog,
# compact feedback output, and removal of the original WASD command path.

from canio import Message
from farm_ng.utils.cobid import CanOpenObject
from farm_ng.utils.main_loop import MainLoop
from farm_ng.utils.packet import AmigaControlState
from farm_ng.utils.packet import AmigaRpdo1
from farm_ng.utils.packet import AmigaTpdo1
from farm_ng.utils.packet import DASHBOARD_NODE_ID
from farm_ng.utils.ticks import TickRepeater
import math
import time
from usb_cdc import console


COMMAND_TIMEOUT_SEC = 0.75
MAX_SERIAL_BUFFER_CHARS = 128
MAX_LINEAR_SPEED = 2.0
MAX_ANGULAR_SPEED = 1.5
# The ROS bridge currently discards this feedback. Leaving it disabled avoids
# any chance that an unconsumed USB transmit buffer blocks the MCU watchdog.
ENABLE_SERIAL_FEEDBACK = False


class HelloMainLoopApp:
    def __init__(self, main_loop: MainLoop, can, node_id) -> None:
        self.can = can
        self.node_id = node_id
        self.main_loop = main_loop
        self.main_loop.show_debug = False
        self.cmd_repeater = TickRepeater(ticks_period_ms=50)

        self.cmd_speed = 0.0
        self.cmd_ang_rate = 0.0
        self.last_command_time = time.monotonic()
        self.serial_buffer = ""
        self.watchdog_triggered = False

        self._register_message_handlers()

    def _register_message_handlers(self):
        cob_id = CanOpenObject.TPDO1 | DASHBOARD_NODE_ID
        self.main_loop.command_handlers[cob_id] = self._handle_amiga_tpdo1

    def _handle_amiga_tpdo1(self, message):
        self.amiga_tpdo1 = AmigaTpdo1.from_can_data(message.data)
        if self.amiga_tpdo1.state != AmigaControlState.STATE_AUTO_ACTIVE:
            self.cmd_speed = 0.0
            self.cmd_ang_rate = 0.0
        if ENABLE_SERIAL_FEEDBACK:
            self.serial_write(
                "{},{},{}".format(
                    self.amiga_tpdo1.state,
                    self.amiga_tpdo1.meas_speed,
                    self.amiga_tpdo1.meas_ang_rate,
                )
            )

    def parse_twist(self, twist):
        fields = twist.strip().split(",")
        if len(fields) != 2:
            raise ValueError("expected two comma-separated fields")
        cmd_speed = float(fields[0])
        cmd_ang_rate = float(fields[1])
        if not math.isfinite(cmd_speed) or not math.isfinite(cmd_ang_rate):
            raise ValueError("command contains NaN or infinity")
        if abs(cmd_speed) > MAX_LINEAR_SPEED:
            raise ValueError("linear command exceeds safety limit")
        if abs(cmd_ang_rate) > MAX_ANGULAR_SPEED:
            raise ValueError("angular command exceeds safety limit")
        self.cmd_speed = cmd_speed
        self.cmd_ang_rate = cmd_ang_rate
        self.last_command_time = time.monotonic()
        if self.watchdog_triggered:
            print("Serial command restored")
        self.watchdog_triggered = False

    def serial_read(self):
        if console.in_waiting > 0:
            chunk = console.read(console.in_waiting).decode()
            self.serial_buffer += chunk
            if len(self.serial_buffer) > MAX_SERIAL_BUFFER_CHARS:
                self.serial_buffer = self.serial_buffer[-MAX_SERIAL_BUFFER_CHARS:]
                print("Serial buffer truncated")

        while "\n" in self.serial_buffer:
            line, self.serial_buffer = self.serial_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                self.parse_twist(line)
            except Exception as exc:
                # Never retain the previous motion command after a malformed or
                # unsafe packet; stop immediately instead of waiting 0.75 s.
                self.cmd_speed = 0.0
                self.cmd_ang_rate = 0.0
                self.watchdog_triggered = True
                print("Bad twist '{}': {}".format(line, exc))

    def serial_write(self, msg):
        console.write(msg.encode() + b"\r\n")

    def enforce_watchdog(self):
        if time.monotonic() - self.last_command_time <= COMMAND_TIMEOUT_SEC:
            return
        self.cmd_speed = 0.0
        self.cmd_ang_rate = 0.0
        if not self.watchdog_triggered:
            print("Serial command watchdog triggered; sending stop")
        self.watchdog_triggered = True

    def iter(self):
        self.serial_read()
        self.enforce_watchdog()
        if self.cmd_repeater.check():
            self.can.send(
                Message(
                    id=CanOpenObject.RPDO1 | DASHBOARD_NODE_ID,
                    data=AmigaRpdo1(
                        state_req=AmigaControlState.STATE_AUTO_ACTIVE,
                        cmd_speed=self.cmd_speed,
                        cmd_ang_rate=self.cmd_ang_rate,
                    ).encode(),
                )
            )


def main():
    MainLoop(AppClass=HelloMainLoopApp, has_display=False).loop()


main()
