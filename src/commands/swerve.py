import commands2
import wpimath.controller
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModuleState

import swervepy

from config.global_options import *


def ski_stop_command(swerve: swervepy.SwerveDrive):
    # fmt: off
    angles = (
        45, 315,  # Front Left, Front Right
        315, 45,  # Back Left, Back Right
    )
    # fmt: on

    states = tuple(SwerveModuleState(0, Rotation2d.fromDegrees(angle)) for angle in angles)

    return commands2.RunCommand(lambda: swerve.desire_module_states(states), swerve)


def drive_command(
    swerve: swervepy.SwerveDrive, x_distance: float, y_distance: float, rotation: float, field_relative: bool = False
):
    translation = Translation2d(x_distance, y_distance)
    return commands2.RunCommand(lambda: swerve.drive(translation, rotation, field_relative, OPEN_LOOP))


class TurnCommand(commands2.Command):
    def __init__(self, swerve: swervepy.SwerveDrive, angle: Rotation2d):
        super().__init__()
        self.swerve = swerve
        self.angle = angle.radians()

        # A feedback controller to control speed
        # Tune the Kp value (first argument) to change how aggressive the controller is
        self.controller = wpimath.controller.PIDController(TURN_CMD_kP, 0, 0)

    def initialize(self):
        self.controller.reset()
        self.controller.setSetpoint(self.angle)

    def execute(self):
        rotational_speed = self.controller.calculate(self.swerve.heading.degrees())
        self.swerve.drive(Translation2d(0, 0), rotational_speed, False, OPEN_LOOP)


def stop_command(swerve: swervepy.SwerveDrive):
    return commands2.InstantCommand(lambda: swerve.drive(Translation2d(0, 0), 0, False, True), swerve)
