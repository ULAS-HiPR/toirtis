import struct
import logging
from typing import Optional

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy import text

from task import Task
from config import ToirtisConfig, RobotParameters

try:
    from scservo_sdk import sms_sts, PortHandler
    import robot.helper
    import robot.presets
    SERVO_AVAILABLE = True
except ImportError:
    SERVO_AVAILABLE = False

#need to update w/ flgith comp stuff
class FlightState:
    UNKNOWN   = 0
    PAD       = 1
    ASCENT    = 2
    DESCENT   = 3
    LANDED    = 4  


FLIGHT_STATE_NAMES = {
    FlightState.UNKNOWN:  "UNKNOWN",
    FlightState.PAD:      "PAD",
    FlightState.ASCENT:   "ASCENT",
    FlightState.DESCENT:  "DESCENT",
    FlightState.LANDED:   "LANDED",
}

FLIGHT_STATE_CAN_ID = 0x100

class RobotTask(Task):
    """
    Reads the most recent FLIGHT_STATE CAN message from the database and
    triggers a walking cycle when landing is detected.

    CAN payload layout (8 bytes):
        [0]      state         uint8   enum
        [1-2]    altitude_m    int16   1 m / LSB
        [3-5]    vspeed_cms    int24   1 cm/s / LSB  (big-endian, signed)
        [6-7]    timestamp_ms  uint16  1 ms / LSB
    """

    def __init__(self, config: ToirtisConfig):
        super().__init__(config)


        self.front_servo_port = None
        self.front_servo      = None
        self.back_servo_port  = None
        self.back_servo       = None
        self.servos_ready     = False
        self._last_flight_state: Optional[int] = None
        self.parameters: Optional[RobotParameters] = None

        for task_config in config.tasks:
            if task_config.name == "robot_task":
                if isinstance(task_config.parameters, RobotParameters):
                    self.parameters = task_config.parameters
                elif isinstance(task_config.parameters, dict):
                    self.parameters = RobotParameters(**task_config.parameters)
                break

        if self.parameters is None:
            raise ValueError("No valid RobotTask configuration found")

    def _init_servos(self, logger: logging.Logger) -> bool:
        if self.servos_ready:
            return True

        if not SERVO_AVAILABLE:
            logger.error("scservo_sdk not available — servo control disabled.")
            return False

        try:
            self.front_servo_port, self.front_servo = helper.make_port_handler(
                self.parameters.front_servo_port
            )
            self.back_servo_port, self.back_servo = helper.make_port_handler(
                self.parameters.back_servo_port
            )

            for sid in self.parameters.front_servo_ids:
                helper.ping_servo(self.front_servo, sid)
                helper.set_limits(self.front_servo, sid, presets.front_servo_min_max[sid])
                helper.enable_torque(self.front_servo, sid)

            for sid in self.parameters.back_servo_ids:
                helper.ping_servo(self.back_servo, sid)
                helper.set_limits(self.back_servo, sid, presets.back_servo_min_max[sid])
                helper.enable_torque(self.back_servo, sid)

            self.servos_ready = True
            logger.info("Servos initialised and torque enabled.")
            return True

        except Exception as e:
            logger.error(f"Servo initialisation failed: {e}")
            return False

    @staticmethod
    def _parse_flight_state(raw_data: list) -> Optional[dict]:
        """
        Parse a FLIGHT_STATE CAN payload from a list of byte ints.
        Returns None if the payload is too short or malformed.
        """
        if not raw_data or len(raw_data) < 8:
            return None

        try:
            data = bytes(raw_data)

            state        = data[0]
            altitude_m   = struct.unpack_from(">h", data, 1)[0]   # int16 big-endian

            # int24 big-endian signed — unpack as 3 bytes then sign-extend
            raw24        = (data[3] << 16) | (data[4] << 8) | data[5]
            vspeed_cms   = raw24 if raw24 < 0x800000 else raw24 - 0x1000000

            timestamp_ms = struct.unpack_from(">H", data, 6)[0]   # uint16 big-endian

            return {
                "state":        state,
                "altitude_m":   altitude_m,
                "vspeed_cms":   vspeed_cms,
                "timestamp_ms": timestamp_ms,
            }

        except Exception:
            return None

    async def _fetch_latest_flight_state(self, engine: AsyncEngine, logger: logging.Logger) -> Optional[dict]:
        """Return the most recent FLIGHT_STATE CAN message, or None."""
        try:
            async with engine.connect() as conn:
                row = await conn.execute(
                    text("""
                        SELECT msg_id, data, created_at
                        FROM   can_messages
                        WHERE  msg_id = :can_id
                        ORDER  BY id DESC
                        LIMIT  1
                    """),
                    {"can_id": FLIGHT_STATE_CAN_ID},
                )
                result = row.fetchone()

            if result is None:
                return None

            import json
            raw_data = json.loads(result.data) if result.data else None
            if raw_data is None:
                return None

            parsed = self._parse_flight_state(raw_data)
            if parsed is None:
                logger.warning("Could not parse FLIGHT_STATE payload.")
                return None

            parsed["received_at"] = str(result.created_at)
            return parsed

        except Exception as e:
            logger.error(f"Failed to read CAN messages from database: {e}")
            return None

    def _do_walk(self, steps: int, logger: logging.Logger) -> None:
        """Run the walking cycle for the given number of steps."""
        import time

        logger.info(f"Starting walking cycle — {steps} step(s).")
        for step in range(steps):
            logger.debug(f"  Step {step + 1}/{steps}")
            for front_pos, back_pos in presets.walking_cycle:
                for sid, pos in front_pos:
                    helper.move_servo(self.front_servo, sid, pos)
                for sid, pos in back_pos:
                    helper.move_servo(self.back_servo, sid, pos)
                time.sleep(0.4)

        logger.info("Walking cycle complete.")

    async def execute(self, engine: AsyncEngine, logger: logging.Logger) -> dict:
        logger.info(f"Executing {self.name} task")

        flight = await self._fetch_latest_flight_state(engine, logger)

        if flight is None:
            logger.info("No FLIGHT_STATE CAN message available yet.")
            return {
                "success": True,
                "action":  "no_data",
                "flight_state": None,
            }

        state      = flight["state"]
        state_name = FLIGHT_STATE_NAMES.get(state, f"UNKNOWN({state})")

        logger.info(
            f"Flight state: {state_name} | "
            f"alt={flight['altitude_m']} m | "
            f"vspeed={flight['vspeed_cms']} cm/s | "
            f"t={flight['timestamp_ms']} ms"
        )

        if state == FlightState.LANDED and self._last_flight_state != FlightState.LANDED:
            logger.info("Landing detected — initiating walking sequence.")

            if not self._init_servos(logger):
                self._last_flight_state = state
                return {
                    "success": False,
                    "action":  "servo_init_failed",
                    "flight_state": state_name,
                    "flight_data":  flight,
                }

            try:
                self._do_walk(steps=4, logger=logger)
                action = "walked"
            except Exception as e:
                logger.error(f"Walking cycle failed: {e}")
                action = "walk_error"

        else:
            if state != self._last_flight_state:
                logger.info(f"State changed to {state_name} — no action required.")
            action = "idle"

        self._last_flight_state = state

        return {
            "success":      True,
            "action":       action,
            "flight_state": state_name,
            "flight_data":  flight,
        }

    def __del__(self):
        if not self.servos_ready:
            return
        try:
            for sid in self.parameters.front_servo_ids:
                helper.disable_torque(self.front_servo, sid)
            for sid in self.parameters.back_servo_ids:
                helper.disable_torque(self.back_servo, sid)
        except Exception:
            pass