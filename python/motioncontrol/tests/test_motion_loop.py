"""Hardware-free checks of dispatch, timing, reporting, and failure cleanup."""

import importlib.util
import io
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from motion_state import MotionHardware, MotionState, MotionTerminal
from datacollection.data_types import CommandResult, LSM9DS1Data


def load_motion_methods():
    """Stub device imports while exercising the real motion loop and helpers."""
    stubs = {name: ModuleType(name) for name in ("adafruit_servokit", "adafruit_motor", "adafruit_motor.servo", "servocontrol", "servocontrol.servo_methods", "motorcontrol", "motorcontrol.roboclaw_methods", "motorcontrol.roboclaw_types")}
    stubs["adafruit_servokit"].ServoKit = object
    stubs["adafruit_motor.servo"].Servo = object
    stubs["servocontrol.servo_methods"].ServoConfig = object
    for name in ("hold_zeroes", "hold_angle", "turn_servos"):
        setattr(stubs["servocontrol.servo_methods"], name, Mock())
    stubs["motorcontrol.roboclaw_methods"].move_motors = Mock()
    stubs["motorcontrol.roboclaw_types"].Motor = object
    stubs["motorcontrol.roboclaw_types"].RoboclawUnit = object
    spec = importlib.util.spec_from_file_location("motion_methods_under_test", Path(__file__).resolve().parents[1] / "motion_methods.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", stubs):
        spec.loader.exec_module(module)
    return module


class MotionLoopTests(unittest.TestCase):
    def setUp(self):
        self.methods = load_motion_methods()
        self.output = io.StringIO()
        self.terminal = MotionTerminal(output_stream=self.output)
        self.controllers = [SimpleNamespace(name=f"controller{i}", address=0x80 + i, unit=SimpleNamespace(SpeedM1M2=Mock(return_value=True))) for i in range(3)]
        self.motors = [SimpleNamespace(id=i, name=f"motor{i}", speed=100, actual_speed=90) for i in range(6)]
        self.hardware = MotionHardware(None, [object()] * 4, [object()] * 4, self.controllers, self.motors, 5, 4, [0, 100, 800])
        self.state = MotionState()
        self.logger = SimpleNamespace(last_error=None, submit=Mock(return_value=True))

    def test_speed_selection_does_not_command_motors(self):
        result = self.methods.shift_speed("t", self.state, self.hardware, self.terminal)
        self.assertEqual(result.selected_speed_qpps, 100)
        self.assertIn("next drive command", self.output.getvalue())
        self.assertTrue(all(motor.speed == 100 for motor in self.motors))
        for controller in self.controllers:
            controller.unit.SpeedM1M2.assert_not_called()
        self.methods.roboclaw_methods.move_motors.assert_not_called()

    def test_speed_selection_clamps_both_ends(self):
        self.methods.shift_speed("r", self.state, self.hardware, self.terminal)
        self.assertEqual(self.state.speed_index, 0)
        for _ in range(5):
            self.methods.shift_speed("t", self.state, self.hardware, self.terminal)
        self.assertEqual(self.state.selected_speed_qpps, 800)

    def test_terminal_caps_and_sanitizes_messages(self):
        self.terminal.report("\x1b[31mError\n" + "x" * 200)
        line = self.output.getvalue().removesuffix("\r\n")
        self.assertLess(len(line), 80)
        self.assertTrue(line.isprintable())
        broken_output = Mock()
        broken_output.write.side_effect = BrokenPipeError()
        MotionTerminal(output_stream=broken_output).report("Stop attempted.")

    def test_poll_is_nonblocking_and_detects_disconnect(self):
        with patch.object(self.terminal, "fileno", return_value=42), patch("motion_state.select.select", return_value=([42], [], [])) as poll, patch("motion_state.os.read", return_value=b""):
            self.assertEqual(self.terminal.poll(), "")
            poll.assert_called_once_with([42], [], [], 0)

    def test_space_stops_without_steering_io(self):
        self.methods.servo_methods.hold_zeroes.side_effect = RuntimeError("steering offline")
        result = self.methods.apply_motion_command(" ", self.state, self.hardware, self.terminal)
        self.assertEqual(result.drive_mode, "stopped")
        self.methods.servo_methods.hold_zeroes.assert_not_called()
        self.methods.servo_methods.hold_angle.assert_not_called()
        for controller in self.controllers:
            controller.unit.SpeedM1M2.assert_called_once_with(controller.address, 0, 0)

    def test_stop_attempts_remaining_controllers_after_failure(self):
        self.controllers[0].unit.SpeedM1M2.side_effect = RuntimeError("disconnected")
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            self.methods.stop_drive(self.state, self.hardware)
        self.assertEqual(self.state.drive_mode, "unknown")
        for controller in self.controllers:
            controller.unit.SpeedM1M2.assert_called_once()

    def test_center_clears_pivot_state_and_stops_before_steering(self):
        self.state.inplace_turn = True
        events = []
        for controller in self.controllers:
            controller.unit.SpeedM1M2.side_effect = lambda *args: events.append("stop") or True
        self.methods.servo_methods.hold_zeroes.side_effect = lambda **kwargs: events.append("center")
        self.methods.apply_motion_command("b", self.state, self.hardware, self.terminal)
        self.assertFalse(self.state.inplace_turn)
        self.assertEqual(events[:3], ["stop"] * 3)
        self.assertEqual(self.state.drive_mode, "stopped")

    def test_command_logged_without_imu_and_repeated_sample_not_duplicated(self):
        result = CommandResult("shift_speed", "ok", "Speed selected.", 100, "stopped")
        self.methods.record_telemetry(self.state, self.hardware, None, self.logger, self.terminal, result)
        self.assertIsNone(self.logger.submit.call_args.args[0])
        self.assertEqual(self.logger.submit.call_args.kwargs["action"], "shift_speed")
        collector = SimpleNamespace(latest=Mock(return_value=LSM9DS1Data(sequence=10)))
        self.methods.record_telemetry(self.state, self.hardware, collector, self.logger, self.terminal)
        self.methods.record_telemetry(self.state, self.hardware, collector, self.logger, self.terminal)
        self.assertEqual(self.logger.submit.call_count, 2)

    def test_unified_drive_commands_preserve_direction_limit_and_watchdog(self):
        for command, mode, signs, ticks in (("w", "forward", [1] * 6, 4), ("s", "reverse", [-1] * 6, 4), ("f", "inplace_left", [-1, 1, -1, 1, -1, 1], 1), ("g", "inplace_right", [1, -1, 1, -1, 1, -1], 1)):
            with self.subTest(command=command):
                state = MotionState(selected_speed_qpps=2000)
                result = self.methods.apply_motion_command(command, state, self.hardware, self.terminal)
                speed = 1000 if command in ("f", "g") else 2000
                self.assertEqual([motor.speed for motor in self.motors], [sign * speed for sign in signs])
                self.assertEqual(result.drive_mode, mode)
                self.assertEqual(state.watchdog_ticks, ticks)

    def test_forward_after_pivot_stops_and_centers_first(self):
        self.state.inplace_turn = True
        self.state.selected_speed_qpps = 100
        events = []
        for controller in self.controllers:
            controller.unit.SpeedM1M2.side_effect = lambda *args: events.append("stop") or True
        self.methods.servo_methods.hold_zeroes.side_effect = lambda **kwargs: events.append("center")
        self.methods.roboclaw_methods.move_motors.side_effect = lambda **kwargs: events.append("drive")
        self.methods.apply_motion_command("w", self.state, self.hardware, self.terminal)
        self.assertEqual(events, ["stop", "stop", "stop", "center", "drive"])
        self.assertFalse(self.state.inplace_turn)

    def test_unified_steering_preserves_key_mapping(self):
        with patch.object(self.methods, "turning_movement", return_value=False) as steer:
            for command, selection in (("a", 1), ("d", 2), ("q", 3), ("e", 4)):
                self.methods.apply_motion_command(command, self.state, self.hardware, self.terminal)
                self.assertEqual(steer.call_args.kwargs["movement_select"], selection)

    def test_shutdown_failure_restores_terminal_and_logs_error(self):
        self.controllers[0].unit.SpeedM1M2.return_value = False
        terminal = Mock(wraps=self.terminal)
        terminal.fileno.return_value = 42
        with patch.object(self.methods.termios, "tcgetattr", return_value="settings"), patch.object(self.methods.termios, "tcsetattr") as restore, patch.object(self.methods.tty, "setraw"):
            with self.assertRaisesRegex(RuntimeError, "stop not acknowledged"):
                self.methods.motion_movement_loop(None, self.hardware.servos, self.hardware.configs, 5, self.controllers, self.motors, terminal=terminal, telemetry_logger=self.logger)
        restore.assert_called_once()
        for controller in self.controllers:
            self.assertEqual(controller.unit.SpeedM1M2.call_count, 2)
        self.methods.servo_methods.hold_zeroes.assert_not_called()
        outcomes = [(item.kwargs["action"], item.kwargs["status"]) for item in self.logger.submit.call_args_list]
        self.assertEqual(outcomes, [("control_error", "error"), ("shutdown", "error")])

    def test_stale_or_failed_imu_does_not_lose_command_result(self):
        result = CommandResult("stop", "ok", "Drive stopped.", 0, "stopped")
        for read in (Mock(return_value=None), Mock(side_effect=OSError("I2C failed"))):
            collector = SimpleNamespace(latest=read)
            self.methods.record_telemetry(self.state, self.hardware, collector, self.logger, self.terminal, result)
            self.assertIsNone(self.logger.submit.call_args.args[0])
            read.assert_called_once_with(max_age_seconds=0.25)
        self.assertEqual(self.logger.submit.call_count, 2)

    def run_loop(self, keys, logger=None):
        terminal = Mock(wraps=self.terminal)
        terminal.fileno.return_value = 42
        terminal.poll.side_effect = keys
        with patch.object(self.methods.termios, "tcgetattr", return_value="settings"), patch.object(self.methods.termios, "tcsetattr") as restore, patch.object(self.methods.tty, "setraw"), patch.object(self.methods.time, "sleep") as sleep:
            self.methods.motion_movement_loop(None, self.hardware.servos, self.hardware.configs, 5, self.controllers, self.motors, speeds=[0, 100], stop_counts=2, terminal=terminal, telemetry_logger=logger)
        return sleep, restore

    def test_loop_paces_buffered_commands_and_logs_watchdog_separately(self):
        sleep, restore = self.run_loop(["t", "w", None, "c"], self.logger)
        self.assertEqual(sleep.call_args_list, [call(0.01)] * 3)
        actions = [item.kwargs["action"] for item in self.logger.submit.call_args_list]
        self.assertEqual(actions, ["watchdog_stop", "shift_speed", "motion", "watchdog_stop", "exit", "shutdown"])
        restore.assert_called_once_with(42, self.methods.termios.TCSADRAIN, "settings")

    def test_disconnect_exits_without_delay(self):
        sleep, restore = self.run_loop([""])
        sleep.assert_not_called()
        restore.assert_called_once()

    def test_failure_is_logged_before_cleanup_and_reported_after_stop(self):
        events = []
        self.logger.submit.side_effect = lambda *args, **kwargs: events.append(kwargs["action"]) or True
        self.methods.roboclaw_methods.move_motors.side_effect = RuntimeError("drive failed")
        self.controllers[0].unit.SpeedM1M2.side_effect = lambda *args: events.append("stop") or True
        self.terminal.report = lambda message: events.append("report")
        with self.assertRaisesRegex(RuntimeError, "drive failed"):
            self.run_loop(["w"], self.logger)
        error_index = events.index("control_error")
        self.assertEqual(events[error_index + 1], "stop")
        self.assertIn("shutdown", events[error_index + 1:])


if __name__ == "__main__":
    unittest.main()
