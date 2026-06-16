import json
import logging
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncEngine

from config import CanParameters, ToirtisConfig
from task import Task


class FlightState:
    UNKNOWN = 0
    PAD = 1
    ASCENT = 2
    DESCENT = 3
    LANDED = 4


FLIGHT_STATE_NAMES = {
    FlightState.UNKNOWN: "UNKNOWN",
    FlightState.PAD: "PAD",
    FlightState.ASCENT: "ASCENT",
    FlightState.DESCENT: "DESCENT",
    FlightState.LANDED: "LANDED",
}

FLIGHT_STATE_CAN_ID = 0x100


@dataclass(frozen=True)
class FlightSample:
    state: int
    altitude_m: int
    vspeed_cms: int


class MockCanTask(Task):
    """Mock CAN task that generates flight-state messages for landing tests."""

    def __init__(self, config: ToirtisConfig, mock_strategy: str = "auto"):
        super().__init__(config)

        self.parameters: Optional[CanParameters] = None

        for task_config in config.tasks:
            if task_config.class_name in ["CanTask", "MockCanTask"] and task_config.parameters is not None:
                if isinstance(task_config.parameters, CanParameters):
                    self.parameters = task_config.parameters
                elif isinstance(task_config.parameters, dict):
                    self.parameters = CanParameters(**task_config.parameters)
                break

        if self.parameters is None:
            self.parameters = CanParameters(listen_timeout=1)

        requested_strategy = mock_strategy
        if requested_strategy == "auto":
            requested_strategy = getattr(self.parameters, "mock_strategy", "landing_sequence")
        self.mock_strategy = self._normalize_strategy(requested_strategy)

        self.start_time = time.time()
        self._sequence_index = 0
        self._landing_sequence: List[FlightSample] = [
            FlightSample(FlightState.PAD, 0, 0),
            FlightSample(FlightState.ASCENT, 120, 1800),
            FlightSample(FlightState.ASCENT, 450, 1400),
            FlightSample(FlightState.DESCENT, 220, -1200),
            FlightSample(FlightState.LANDED, 0, 0),
        ]

    def _normalize_strategy(self, strategy: str) -> str:
        if strategy in ["auto", "landing_sequence", "sequence"]:
            return "landing_sequence"
        if strategy in ["landing", "landed", "static"]:
            return "landed"
        return strategy

    def _encode_int24(self, value: int) -> List[int]:
        if value < 0:
            value = (1 << 24) + value
        return [(value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF]

    def _build_payload(self, sample: FlightSample) -> List[int]:
        timestamp_ms = int((time.time() - self.start_time) * 1000) & 0xFFFF
        altitude_bytes = [(sample.altitude_m >> 8) & 0xFF, sample.altitude_m & 0xFF]
        speed_bytes = self._encode_int24(sample.vspeed_cms)
        timestamp_bytes = [(timestamp_ms >> 8) & 0xFF, timestamp_ms & 0xFF]

        return [sample.state, *altitude_bytes, *speed_bytes, *timestamp_bytes]

    def _next_sample(self) -> FlightSample:
        if self.mock_strategy == "landed":
            return FlightSample(FlightState.LANDED, 0, 0)

        index = min(self._sequence_index, len(self._landing_sequence) - 1)
        sample = self._landing_sequence[index]
        if self._sequence_index < len(self._landing_sequence) - 1:
            self._sequence_index += 1
        return sample

    def _generate_message(self) -> Dict[str, object]:
        sample = self._next_sample()
        payload = self._build_payload(sample)

        return {
            "msg_id": FLIGHT_STATE_CAN_ID,
            "msg_id_hex": hex(FLIGHT_STATE_CAN_ID),
            "is_rtr": False,
            "data": payload,
            "rtr_length": None,
            "flight_state": FLIGHT_STATE_NAMES.get(sample.state, f"UNKNOWN({sample.state})"),
            "altitude_m": sample.altitude_m,
            "vspeed_cms": sample.vspeed_cms,
            "timestamp_ms": payload[-2] << 8 | payload[-1],
        }

    async def execute(self, engine: AsyncEngine, logger: logging.Logger) -> dict:
        logger.info(f"Executing mock {self.name} task (strategy: {self.mock_strategy})")

        message = self._generate_message()

        try:
            async with engine.connect() as conn:
                await conn.execute(
                    text(
                        """
                        INSERT INTO can_messages
                            (msg_id, msg_id_hex, is_rtr, data, rtr_length, sensor_config)
                        VALUES
                            (:msg_id, :msg_id_hex, :is_rtr, :data, :rtr_length, :config)
                        """
                    ),
                    {
                        "msg_id": message["msg_id"],
                        "msg_id_hex": message["msg_id_hex"],
                        "is_rtr": message["is_rtr"],
                        "data": json.dumps(message["data"]),
                        "rtr_length": message["rtr_length"],
                        "config": json.dumps(
                            {
                                "mock_strategy": self.mock_strategy,
                                "spi_bus": self.parameters.spi_bus,
                                "cs_pin": str(self.parameters.cs_pin),
                                "loopback": self.parameters.loopback,
                                "silent": self.parameters.silent,
                                "flight_state": message["flight_state"],
                            }
                        ),
                    },
                )
                await conn.commit()

            logger.info(
                f"Stored mock CAN message: state={message['flight_state']} "
                f"alt={message['altitude_m']} m vspeed={message['vspeed_cms']} cm/s"
            )

            return {
                "success": True,
                "error": None,
                "data": [message],
                "mock_strategy": self.mock_strategy,
                "flight_state": message["flight_state"],
            }

        except Exception as e:
            error_msg = f"Failed to store mock CAN data in database: {e}"
            logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
                "data": [message],
                "mock_strategy": self.mock_strategy,
            }