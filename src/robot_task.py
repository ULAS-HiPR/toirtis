import asyncio
import logging
from typing import Optional

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy import text

from task import Task
from config import ToirtisConfig, RobotParameters

try:
    from scservo_sdk import sms_sts, PortHandler
    from robot import helper, presets
    SERVO_AVAILABLE = True
except ImportError:
    SERVO_AVAILABLE = False

LIFTOFF_ACCEL_THRESHOLD = 40.0  # m/s² — abs(accel_x) + abs(accel_y) + abs(accel_z)
WALK_DELAY_S = 180               # 3 minutes after liftoff detected


class RobotTask(Task):
    """
    Monitors raw IMU data. Once the sum of absolute acceleration axes
    exceeds LIFTOFF_ACCEL_THRESHOLD, starts a 3-minute countdown then
    triggers the walking sequence.
    """

    def __init__(self, config: ToirtisConfig):
        super().__init__(config)

        self.front_servo_port = None
        self.front_servo      = None
        self.back_servo_port  = None
        self.back_servo       = None
        self.servos_ready     = False
        self._walk_done       = False
        self._liftoff_time: Optional[float] = None   # monotonic time of liftoff
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

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------

    async def _fetch_latest_imu(self, engine: AsyncEngine, logger: logging.Logger) -> Optional[dict]:
        """Return the most recent imu_readings row, or None."""
        try:
            async with engine.connect() as conn:
                row = await conn.execute(
                    text("""
                        SELECT accel_x, accel_y, accel_z,
                               gyro_x, gyro_y, gyro_z, timestamp
                        FROM   imu_readings
                        ORDER  BY id DESC
                        LIMIT  1
                    """)
                )
                result = row.fetchone()

            if result is None:
                return None

            return {
                "accel_x":   result.accel_x,
                "accel_y":   result.accel_y,
                "accel_z":   result.accel_z,
                "gyro_x":    result.gyro_x,
                "gyro_y":    result.gyro_y,
                "gyro_z":    result.gyro_z,
                "timestamp": str(result.timestamp),
            }

        except Exception as e:
            logger.error(f"Failed to read IMU data from database: {e}")
            return None

    # ------------------------------------------------------------------
    # Walk helpers
    # ------------------------------------------------------------------

    def _run_walk_and_pose(self, logger: logging.Logger) -> None:
        if not self._init_servos(logger):
            return
        self._do_walk(55, logger)
        self._do_can_pose()

    async def _run_walk_sequence(self, logger: logging.Logger) -> None:
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

        print("\nMoving to can pose")
        for sid, pos in presets.front_can_pos:
            helper.move_servo(self.front_servo, sid, pos)
        for sid, pos in presets.back_can_pos:
            helper.move_servo(self.back_servo, sid, pos)
        time.sleep(2)

    # ------------------------------------------------------------------
    # Servo init
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Walking
    # ------------------------------------------------------------------

    def _do_walk(self, steps: int, logger: logging.Logger) -> None:
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

    # ------------------------------------------------------------------
    # Main execute
    # ------------------------------------------------------------------

    async def execute(self, engine: AsyncEngine, logger: logging.Logger) -> dict:
        logger.info(f"Executing {self.name} task")

        imu = await self._fetch_latest_imu(engine, logger)

        if imu is None:
            logger.info("No IMU data available yet.")
            return {"success": True, "action": "no_data", "imu": None}

        accel_magnitude = abs(imu["accel_x"]) + abs(imu["accel_y"]) + abs(imu["accel_z"])

        logger.info(
            f"IMU accel — x={imu['accel_x']:.2f}  y={imu['accel_y']:.2f}  "
            f"z={imu['accel_z']:.2f}  |total|={accel_magnitude:.2f} m/s²"
        )

        # ------------------------------------------------------------------
        # Detect liftoff from raw IMU and stamp the time (once only)
        # ------------------------------------------------------------------
        if self._liftoff_time is None and accel_magnitude > LIFTOFF_ACCEL_THRESHOLD:
            self._liftoff_time = asyncio.get_event_loop().time()
            logger.info(
                f"Liftoff detected! |accel|={accel_magnitude:.2f} m/s² — "
                f"walk will fire in {WALK_DELAY_S}s ({WALK_DELAY_S/60:.1f} min)."
            )
            print(f"Liftoff detected — walk timer started ({WALK_DELAY_S}s).")

        # ------------------------------------------------------------------
        # Harvest any finished background walk task
        # ------------------------------------------------------------------
        if self._walk_task is not None and self._walk_task.done():
            try:
                self._walk_task.result()
            except Exception as e:
                logger.error(f"Walking task failed: {e}")
            finally:
                self._walk_task = None

        # ------------------------------------------------------------------
        # Fire walk once the delay has elapsed
        # ------------------------------------------------------------------
        action = "idle"

        if self._liftoff_time is not None and not self._walk_done:
            elapsed   = asyncio.get_event_loop().time() - self._liftoff_time
            remaining = WALK_DELAY_S - elapsed

            if remaining <= 0:
                if self._walk_task is None:
                    logger.info("3-minute post-liftoff delay elapsed — starting walk.")
                    print("3-minute post-liftoff delay elapsed — starting walk.")
                    self._walk_task = asyncio.create_task(self._run_walk_sequence(logger))
                    action = "walking_started"
                else:
                    action = "walking"
            else:
                logger.debug(f"Walk countdown: {remaining:.0f}s remaining.")
                action = f"waiting_{remaining:.0f}s"

        return {
            "success":          True,
            "action":           action,
            "accel_magnitude":  round(accel_magnitude, 3),
            "imu":              imu,
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