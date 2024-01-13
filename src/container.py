import commands2
import wpilib

from swervepy import SwerveDrive

from config import switchable_options
from commands.swerve import ski_stop_command
from config.global_options import *
from oi import XboxDriver


class RobotContainer:
    def __init__(self):
        # Driver Xbox controller
        self.stick = XboxDriver(DRIVER_JOYSTICK)

        # Load configs for the specific robot this code is deployed to
        # Determined by a value set in the ROBOT_ID file
        options = switchable_options.get_robot_specific_options()

        # Construct the swerve drivetrain
        self.swerve = SwerveDrive(options.MODULES, options.GYRO, options.MAX_VELOCITY, options.MAX_ANGULAR_VELOCITY)

        self.teleop_command = self.swerve.teleop_command(
            self.stick.forward,
            self.stick.strafe,
            self.stick.turn,
            FIELD_RELATIVE,
            OPEN_LOOP,
        )
        # The teleop command will run whenever no other command is running on the Swerve subsystem
        # (e.g., autonomous, ski stop)
        self.swerve.setDefaultCommand(self.teleop_command)
        wpilib.SmartDashboard.putData(self.teleop_command)

        self.configure_button_bindings()

    def get_autonomous_command(self):
        return commands2.PrintCommand("Autonomous not implemented!")

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        # Reset the robot's heading reading to 0
        self.stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.stick.toggle_field_relative.onTrue(commands2.InstantCommand(self.teleop_command.toggle_field_relative))

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.stick.is_movement_commanded))
