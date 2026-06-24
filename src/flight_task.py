import time
import logging
from typing import Optional

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy import text

from task import Task
from config import ToirtisConfig, FlightParameters
from fsm.state_machine import FlightStateMachine, State
from fsm.kalman_filter import KalmanFilter


class FlightTask(Task):
    """Fuses the latest barometer + IMU readings into a flight-state estimate."""

    def __init__(self, config: ToirtisConfig):
        super().__init__(config)

        self.parameters: Optional[FlightParameters] = None
        for task_config in config.tasks:
            if task_config.class_name == "FlightTask" and task_config.name == getattr(self, "task_name", "flight"):
                self.parameters = task_config.parameters
                break
        if self.parameters is None:
            self.parameters = FlightParameters()

        self.kf = KalmanFilter()
        self.sm = FlightStateMachine(
            main_height=self.parameters.main_height,
            liftoff_threshold=self.parameters.liftoff_threshold,
            drouge_delay=self.parameters.drouge_delay,
        )
        self._last_time: Optional[float] = None

    async def _fetch_latest_baro(self, engine: AsyncEngine):
        async with engine.connect() as conn:
            row = await conn.execute(text(
                "SELECT timestamp, altitude_meters FROM barometer_readings ORDER BY id DESC LIMIT 1"
            ))
            return row.fetchone()

    async def _fetch_latest_imu(self, engine: AsyncEngine):
        # accel_axis is validated against a fixed whitelist in FlightParameters, safe to interpolate
        async with engine.connect() as conn:
            row = await conn.execute(text(
                f"SELECT timestamp, {self.parameters.accel_axis} AS accel FROM imu_readings ORDER BY id DESC LIMIT 1"
            ))
            return row.fetchone()

    async def execute(self, engine: AsyncEngine, logger: logging.Logger) -> dict:
        logger.info(f"Executing {self.name} task")

        baro = await self._fetch_latest_baro(engine)
        imu = await self._fetch_latest_imu(engine)

        if baro is None or imu is None or baro.altitude_meters is None or imu.accel is None:
            logger.info("Waiting for valid baro/IMU data before fusing flight state.")
            return {"success": True, "status": "no_data", "state": None}

        now = time.time()
        accel = imu.accel - self.parameters.gravity_offset

        if self._last_time is None:
            self._last_time = now
            return {"success": True, "status": "warming_up", "state": None}

        dt = max(0.001, min(now - self._last_time, 0.5))
        self._last_time = now

        self.kf.predict(dt)
        self.kf.update(z_baro=baro.altitude_meters, z_accel=accel)
        pred = self.kf.get()

        self.sm.update(pred)
        state = self.sm.state
        state_name = state.name

        try:
            async with engine.connect() as conn:
                await conn.execute(
                    text("""
                        INSERT INTO flight_state
                            (state, state_name, altitude_m, velocity_mps, acceleration_mps2,
                             raw_altitude_m, raw_accel)
                        VALUES
                            (:state, :state_name, :altitude_m, :velocity_mps, :acceleration_mps2,
                             :raw_altitude_m, :raw_accel)
                    """),
                    {
                        "state": int(state),
                        "state_name": state_name,
                        "altitude_m": pred["altitude"],
                        "velocity_mps": pred["velocity"],
                        "acceleration_mps2": pred["acceleration"],
                        "raw_altitude_m": baro.altitude_meters,
                        "raw_accel": imu.accel,
                    },
                )
                await conn.commit()
        except Exception as e:
            logger.error(f"Failed to store flight state: {e}")
            return {"success": False, "error": str(e), "state": state_name}

        logger.info(
            f"Flight state: {state_name} | alt={pred['altitude']:.1f} m | "
            f"vel={pred['velocity']:.2f} m/s | accel={pred['acceleration']:.2f} m/s²"
        )
        return {"success": True, "state": state_name, "prediction": pred}