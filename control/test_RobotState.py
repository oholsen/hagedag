import RobotMessages
import pytest


def test_move_cmd():
    move = RobotMessages.MoveCommand(0.12, 0.5, 24.125)
    assert str(move) == "m 2153.71 1480.68 24.125"


def test_move_ack1():
    move = RobotMessages.MoveCommand(0.12, 0.5, 24.125)
    line = "Control " + str(move)
    c = RobotMessages.process(line)
    assert isinstance(c, RobotMessages.MoveAck)
    assert c.timeout == 24.125
    assert c.speed == pytest.approx(move.speed, 1e-3)
    assert c.omega == pytest.approx(move.omega, 1e-3)
    assert c.timeout == move.timeout


def test_move_ack2():
    move = RobotMessages.MoveCommand(0.12, 0.5, 24.125)
    line = "Ack 23.123 " + str(move)
    assert line == "Ack 23.123 " + str(move)
    x = RobotMessages.process(line)
    assert isinstance(x, RobotMessages.Ack)
    assert x.time == 23.123
    c = x.command
    assert isinstance(c, RobotMessages.MoveAck)
    assert c.speed == pytest.approx(move.speed, 1e-3)
    assert c.omega == pytest.approx(move.omega, 1e-3)
    assert c.timeout == move.timeout



def test_power():
    line = "Power 0.8 0.9"
    x = RobotMessages.process(line)
    assert isinstance(x, RobotMessages.Power)
    assert x.left == pytest.approx(0.8, 1e-3)
    assert x.right == pytest.approx(0.9, 1e-3)

