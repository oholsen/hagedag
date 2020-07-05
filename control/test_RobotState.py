import RobotState
import pytest


def test_move():
    move = RobotState.MoveCommand(0.12, 0.23, 24.125)
    line = "Ack 23.123 " + str(move)
    assert line == "Ack 23.123 m 12.00 -13.18 24.125"
    x = RobotState.process(line)
    assert isinstance(x, RobotState.Ack)
    assert x.time == 23.123
    c = x.command
    assert isinstance(c, RobotState.Move)
    assert c.speed == move.speed
    assert c.omega == pytest.approx(move.omega, 1e-3)
    assert c.timeout == move.timeout
