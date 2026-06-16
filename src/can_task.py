import json
import asyncio
from typing import Dict, List, Optional
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy import text
import logging

from task import Task
from config import ToirtisConfig, CanParameters

try:
    import board
    import busio
    from adafruit_mcp2515 import MCP2515
    from adafruit_mcp2515.canio import Message, RemoteTransmissionRequest
    SENSOR_AVAILABLE = True
except ImportError:
    SENSOR_AVAILABLE = False


class CanTask(Task):
    """Task for reading CAN messages from an MCP2515 controller."""

    def __init__(self, config: ToirtisConfig):
        super().__init__(config)
        self.can_bus = None
        self.spi = None
        self.initialized = False
        self.parameters: Optional[CanParameters] = None

        for task_config in config.tasks:
            if task_config.class_name == "CanTask" and task_config.name == getattr(self, 'task_name', 'can'):
                self.parameters = task_config.parameters
                break

        if self.parameters is None:
            self.parameters = CanParameters()

    async def _initialize_sensor(self, logger: logging.Logger) -> bool:
        """Initialize the MCP2515 CAN controller."""
        if not SENSOR_AVAILABLE:
            logger.error(
                "CAN libraries not available. "
                "Install adafruit-blinka and adafruit-circuitpython-mcp2515."
            )
            return False

        if self.initialized:
            return True

        try:
            if self.parameters.spi_bus == 1:
                self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
            else:
                logger.error(
                    f"SPI bus {self.parameters.spi_bus} not supported. Only bus 1 is supported."
                )
                return False

            self.can_bus = MCP2515(
                self.spi,
                cs=self.parameters.cs_pin,
                loopback=self.parameters.loopback,
                silent=self.parameters.silent,
            )

            self.initialized = True
            logger.info(
                f"MCP2515 initialized on SPI bus {self.parameters.spi_bus}, "
                f"CS pin {self.parameters.cs_pin}"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize MCP2515: {e}")
            self.initialized = False
            return False

    def _read_can_messages(self, logger: logging.Logger) -> List[Dict]:
        """
        Read all waiting CAN messages from the bus using a 1-second listen window.
        Returns a list of dicts, one per message.
        """
        if not self.initialized or not self.can_bus:
            logger.warning("CAN bus not initialized.")
            return []

        messages = []

        try:
            with self.can_bus.listen(timeout=1.0) as listener:
                message_count = listener.in_waiting()
                logger.debug(f"{message_count} CAN message(s) waiting.")

                for _ in range(message_count):
                    msg = listener.receive()

                    if msg is None:
                        continue

                    entry = {
                        "msg_id": msg.id,
                        "msg_id_hex": hex(msg.id),
                        "is_rtr": False,
                        "data": None,
                        "rtr_length": None,
                    }

                    if isinstance(msg, Message):
                        entry["data"] = list(msg.data) 
                        logger.debug(f"CAN message  id={hex(msg.id)}  data={list(msg.data)}")
                    elif isinstance(msg, RemoteTransmissionRequest):
                        entry["is_rtr"] = True
                        entry["rtr_length"] = msg.length
                        logger.debug(f"CAN RTR  id={hex(msg.id)}  length={msg.length}")

                    messages.append(entry)

        except Exception as e:
            logger.error(f"Error reading CAN messages: {e}")

        return messages

    async def execute(self, engine: AsyncEngine, logger: logging.Logger) -> dict:
        """Execute the CAN reading task."""
        logger.info(f"Executing {self.name} task")

        if not await self._initialize_sensor(logger):
            error_msg = "Failed to initialize CAN bus"
            logger.error(error_msg)
            return {"success": False, "error": error_msg, "data": None}

        messages = self._read_can_messages(logger)

        if not messages:
            logger.info("No CAN messages received this cycle.")
            return {"success": True, "error": None, "data": []}

        try:
            async with engine.connect() as conn:
                for msg in messages:
                    await conn.execute(
                        text("""
                            INSERT INTO can_messages
                                (msg_id, msg_id_hex, is_rtr, data, rtr_length, sensor_config)
                            VALUES
                                (:msg_id, :msg_id_hex, :is_rtr, :data, :rtr_length, :config)
                        """),
                        {
                            "msg_id": msg["msg_id"],
                            "msg_id_hex": msg["msg_id_hex"],
                            "is_rtr": msg["is_rtr"],
                            "data": json.dumps(msg["data"]),
                            "rtr_length": msg["rtr_length"],
                            "config": json.dumps({
                                "spi_bus": self.parameters.spi_bus,
                                "cs_pin": str(self.parameters.cs_pin),
                                "loopback": self.parameters.loopback,
                                "silent": self.parameters.silent,
                            }),
                        },
                    )
                await conn.commit()

            logger.info(f"Stored {len(messages)} CAN message(s).")
            return {"success": True, "error": None, "data": messages}

        except Exception as e:
            error_msg = f"Failed to store CAN messages in database: {e}"
            logger.error(error_msg)
            return {"success": False, "error": error_msg, "data": messages}

    def __del__(self):
        """Cleanup SPI resources."""
        if self.spi:
            try:
                self.spi.deinit()
            except Exception:
                pass