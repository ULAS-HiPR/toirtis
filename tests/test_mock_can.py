import json
import os
import sys
from datetime import datetime
from unittest.mock import AsyncMock, Mock, patch

import pytest
from sqlalchemy.ext.asyncio import AsyncEngine

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from config import MachaConfig
from mock_can_task import FLIGHT_STATE_CAN_ID, FlightState, MockCanTask
from robot_task import RobotTask


@pytest.fixture
def mock_engine():
    engine = Mock(spec=AsyncEngine)
    mock_conn = AsyncMock()
    mock_conn.execute = AsyncMock()
    mock_conn.commit = AsyncMock()
    mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
    mock_conn.__aexit__ = AsyncMock(return_value=None)
    engine.connect.return_value = mock_conn
    return engine


@pytest.fixture
def mock_logger():
    logger = Mock()
    logger.info = Mock()
    logger.warning = Mock()
    logger.error = Mock()
    logger.debug = Mock()
    return logger


@pytest.fixture
def mock_can_config():
    return MachaConfig(
        app={"name": "test", "debug": True},
        logging={"level": "INFO", "file": {"path": "test.log"}, "console": {"format": "test"}},
        db={"filename": "test.db", "connection_string": "sqlite:///test.db", "overwrite": False},
        tasks=[
            {
                "name": "mock_can_monitor",
                "class": "MockCanTask",
                "frequency": 5,
                "enabled": True,
                "parameters": {
                    "spi_bus": 1,
                    "cs_pin": 0,
                    "loopback": False,
                    "silent": False,
                    "listen_timeout": 1,
                    "mock_strategy": "landed",
                },
            }
        ],
    )


@pytest.fixture
def robot_config():
    return MachaConfig(
        app={"name": "test", "debug": True},
        logging={"level": "INFO", "file": {"path": "test.log"}, "console": {"format": "test"}},
        db={"filename": "test.db", "connection_string": "sqlite:///test.db", "overwrite": False},
        tasks=[
            {
                "name": "robot_task",
                "class": "RobotTask",
                "frequency": 5,
                "enabled": True,
                "parameters": {
                    "front_servo_port": "/dev/ttyUSB0",
                    "back_servo_port": "/dev/ttyUSB1",
                    "front_servo_ids": [1, 2],
                    "back_servo_ids": [3, 4],
                },
            }
        ],
    )


class TestMockCanTask:
    def test_initialization_uses_mock_strategy(self, mock_can_config):
        task = MockCanTask(mock_can_config)

        assert task.name == "MockCanTask"
        assert task.mock_strategy == "landed"
        assert task.parameters.mock_strategy == "landed"

    @pytest.mark.asyncio
    async def test_execute_stores_landed_message(self, mock_can_config, mock_engine, mock_logger):
        task = MockCanTask(mock_can_config, mock_strategy="landed")

        result = await task.execute(mock_engine, mock_logger)

        assert result["success"] is True
        assert result["flight_state"] == "LANDED"
        assert result["data"][0]["msg_id"] == FLIGHT_STATE_CAN_ID
        assert result["data"][0]["data"][0] == FlightState.LANDED

        mock_conn = mock_engine.connect.return_value.__aenter__.return_value
        assert mock_conn.execute.call_count == 1
        params = mock_conn.execute.call_args.args[1]
        assert params["msg_id"] == FLIGHT_STATE_CAN_ID
        assert json.loads(params["data"])[0] == FlightState.LANDED


class TestRobotTaskLandingReaction:
    def test_robot_task_does_not_move_on_initialization(self, robot_config):
        with patch.object(RobotTask, "_init_servos") as mock_init_servos, patch.object(
            RobotTask, "_do_can_pose"
        ) as mock_do_can_pose:
            task = RobotTask(robot_config)

        assert task.servos_ready is False
        assert task._walk_task is None
        assert task._landing_armed is False
        mock_init_servos.assert_not_called()
        mock_do_can_pose.assert_not_called()

    @pytest.mark.asyncio
    async def test_robot_ignores_initial_landed_state(self, robot_config, mock_engine, mock_logger):
        task = RobotTask(robot_config)

        landed_payload = [
            FlightState.LANDED,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x2A,
        ]

        row = Mock()
        row.data = json.dumps(landed_payload)
        row.created_at = datetime.utcnow()

        result_set = Mock()
        result_set.fetchone.return_value = row

        mock_conn = mock_engine.connect.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock(return_value=result_set)

        task._init_servos = Mock(return_value=True)
        task._do_walk = Mock()
        task._do_can_pose = Mock()

        result = await task.execute(mock_engine, mock_logger)

        assert result["success"] is True
        assert result["action"] == "idle"
        assert result["flight_state"] == "LANDED"
        assert task._landing_armed is False
        assert task._walk_task is None
        task._init_servos.assert_not_called()
        task._do_walk.assert_not_called()

    @pytest.mark.asyncio
    async def test_robot_walks_after_first_landed_message(self, robot_config, mock_engine, mock_logger):
        task = RobotTask(robot_config)
        events = []

        arm_payload = [
            FlightState.ASCENT,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x2A,
        ]

        arm_row = Mock()
        arm_row.data = json.dumps(arm_payload)
        arm_row.created_at = datetime.utcnow()

        arm_result_set = Mock()
        arm_result_set.fetchone.return_value = arm_row

        landing_payload = [
            FlightState.LANDED,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x2A,
        ]

        landing_row = Mock()
        landing_row.data = json.dumps(landing_payload)
        landing_row.created_at = datetime.utcnow()

        landing_result_set = Mock()
        landing_result_set.fetchone.return_value = landing_row

        mock_conn = mock_engine.connect.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock(side_effect=[arm_result_set, landing_result_set])

        task._init_servos = Mock(return_value=True)
        task._do_walk = Mock(side_effect=lambda steps, logger: events.append("walk"))
        task._do_can_pose = Mock(side_effect=lambda: events.append("pose"))

        async def inline_to_thread(func, *args, **kwargs):
            return func(*args, **kwargs)

        with patch("robot_task.asyncio.to_thread", side_effect=inline_to_thread):
            arm_result = await task.execute(mock_engine, mock_logger)
            assert arm_result["action"] == "idle"
            assert task._landing_armed is True

            result = await task.execute(mock_engine, mock_logger)

        assert result["success"] is True
        assert result["action"] == "walking_started"
        assert result["flight_state"] == "LANDED"
        task._init_servos.assert_not_called()

        assert task._walk_task is not None
        await task._walk_task

        task._init_servos.assert_called_once_with(mock_logger)
        task._do_walk.assert_called_once_with(55, mock_logger)
        task._do_can_pose.assert_called_once_with()
        assert events == ["walk", "pose"]

    @pytest.mark.asyncio
    async def test_robot_resets_walk_flag_after_departing_landing(self, robot_config, mock_engine, mock_logger):
        task = RobotTask(robot_config)
        task._walk_done = True
        task._landing_armed = True

        departure_payload = [
            FlightState.ASCENT,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x2A,
        ]

        row = Mock()
        row.data = json.dumps(departure_payload)
        row.created_at = datetime.utcnow()

        result_set = Mock()
        result_set.fetchone.return_value = row

        mock_conn = mock_engine.connect.return_value.__aenter__.return_value
        mock_conn.execute = AsyncMock(return_value=result_set)

        task._init_servos = Mock(return_value=True)
        task._do_walk = Mock()

        result = await task.execute(mock_engine, mock_logger)

        assert result["success"] is True
        assert result["action"] == "idle"
        assert task._walk_done is False
        task._init_servos.assert_not_called()
        task._do_walk.assert_not_called()