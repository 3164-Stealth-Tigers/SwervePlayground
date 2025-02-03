import traceback

import commands2
import wpilib
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder

from swervepy import SwerveDrive, TrajectoryFollowerParameters

from pathplannerlib.path import PathPlannerPath
from config import switchable_options
from commands.swerve import ski_stop_command
from config.global_options import *
from oi import XboxDriver, PS4Driver


class RobotContainer:
    def __init__(self):
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # Driver Xbox controller
        self.stick = PS4Driver(DRIVER_JOYSTICK)

        # Load configs for the specific robot this code is deployed to
        # Determined by a value set in the ROBOT_ID file
        self.options = switchable_options.get_robot_specific_options()

        # Construct the swerve drivetrain
        self.swerve = SwerveDrive(
            self.options.MODULES,
            self.options.GYRO,
            self.options.MAX_VELOCITY,
            self.options.MAX_ANGULAR_VELOCITY,
            TrajectoryFollowerParameters(5, 5, OPEN_LOOP),
        )

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

        # Load PathPlanner autos
        self.auto = commands2.Command()
        try:
            path = PathPlannerPath.fromPathFile("Around")
            self.auto = AutoBuilder.followPath(path).beforeStarting(
                AutoBuilder.resetOdom(path.getStartingHolonomicPose())
            )
        except RuntimeError:
            print(traceback.format_exc())

        # Setup SysId
        self.sysid_chooser = wpilib.SendableChooser()
        self.sysid_chooser.addOption(
            "Quasistatic Forward", self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        self.sysid_chooser.addOption(
            "Quasistatic Reverse", self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.sysid_chooser.addOption("Dynamic Forward", self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.sysid_chooser.addOption("Dynamic Reverse", self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        wpilib.SmartDashboard.putData("SysId Chooser", self.sysid_chooser)

    def get_autonomous_command(self):
        return self.auto

    def get_test_command(self):
        # Return a SysId routine command
        return self.sysid_chooser.getSelected()

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        # Reset the robot's heading reading to 0
        self.stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.stick.toggle_field_relative.onTrue(commands2.InstantCommand(self.teleop_command.toggle_field_relative))

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.stick.is_movement_commanded))
