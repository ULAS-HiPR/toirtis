import asyncio
import logging
from typing import Optional

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy import text

from task import Task
from config import ToirtisConfig, RobotParameters
from fsm.state_machine import State

try:
    from scservo_sdk import sms_sts, PortHandler
    from robot import helper, presets
    SERVO_AVAILABLE = True
except ImportError:
    SERVO_AVAILABLE = False


class RobotTask(Task):
    """
    Reads the most recent flight_state row from the database and
    triggers a walking cycle when landing is detected.
    """

    def __init__(self, config: ToirtisConfig):
        super().__init__(config)

        self.front_servo_port = None
        self.front_servo      = None
        self.back_servo_port  = None
        self.back_servo       = None
        self.servos_ready     = False
        self._last_flight_state: Optional[int] = None
        self._walk_done = False
        self._landing_armed = False
        self._walk_task: Optional[asyncio.Task] = None
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

    def _run_walk_and_pose(self, logger: logging.Logger) -> None:
        if not self._init_servos(logger):
            return

        self._do_walk(55, logger)
        self._do_can_pose()

    async def _run_walk_sequence(self, logger: logging.Logger) -> None:
        """Run the landing walk without blocking the event loop."""
        try:
            await asyncio.to_thread(self._run_walk_and_pose, logger)
            self._walk_done = True
        except Exception as e:
            logger.error(f"Background walking sequence failed: {e}")
        finally:
            self._walk_task = None

    def _do_can_pose(self) -> None:
        import time
        for sid, pos in presets.front_legs_pos:
            helper.move_servo(self.front_servo, sid, pos)
        for sid, pos in presets.back_legs_pos:
            helper.move_servo(self.back_servo, sid, pos)

        time.sleep(2)

        print("\nMoving to can")
        for sid, pos in presets.front_can_pos:
            helper.move_servo(self.front_servo, sid, pos)
        for sid, pos in presets.back_can_pos:
            helper.move_servo(self.back_servo, sid, pos)

        time.sleep(2)

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

    async def _fetch_latest_flight_state(self, engine: AsyncEngine, logger: logging.Logger) -> Optional[dict]:
        """Return the most recent flight_state row, or None."""
        try:
            async with engine.connect() as conn:
                row = await conn.execute(
                    text("""
                        SELECT state, state_name, altitude_m, velocity_mps,
                               acceleration_mps2, timestamp
                        FROM   flight_state
                        ORDER  BY id DESC
                        LIMIT  1
                    """)
                )
                result = row.fetchone()

            if result is None:
                return None

            return {
                "state":            result.state,
                "state_name":       result.state_name,
                "altitude_m":       result.altitude_m,
                "velocity_mps":     result.velocity_mps,
                "acceleration_mps2": result.acceleration_mps2,
                "received_at":      str(result.timestamp),
            }

        except Exception as e:
            logger.error(f"Failed to read flight state from database: {e}")
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
            logger.info("No flight state available yet.")
            return {
                "success": True,
                "action":  "no_data",
                "flight_state": None,
            }

        state      = flight["state"]
        state_name = flight["state_name"]

        logger.info(
            f"Flight state: {state_name} | "
            f"alt={flight['altitude_m']:.1f} m | "
            f"vel={flight['velocity_mps']:.2f} m/s | "
            f"accel={flight['acceleration_mps2']:.2f} m/s²"
        )

        if self._walk_task is not None and self._walk_task.done():
            try:
                self._walk_task.result()
            except Exception as e:
                logger.error(f"Walking task failed: {e}")
            finally:
                self._walk_task = None

        if state != State.LANDED and self._walk_done:
            self._walk_done = False

        if state != State.LANDED:
            self._landing_armed = True

        if state == State.LANDED and self._landing_armed and not self._walk_done:
            logger.info("Landing detected — initiating walking sequence.")

            if self._walk_task is None:
                self._walk_task = asyncio.create_task(self._run_walk_sequence(logger))
                action = "walking_started"
            else:
                action = "walking"

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