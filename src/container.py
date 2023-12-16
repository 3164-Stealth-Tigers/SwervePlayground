import math

import commands2
import wpilib
import wpimath.trajectory
from wpimath.geometry import Pose2d

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters

import switchable_options
from commands.swerve import ski_stop_command
from global_options import *
from oi import XboxDriver, T16000M


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
        follower_params = TrajectoryFollowerParameters(
            target_angular_velocity=math.pi * (u.rad / u.s),
            target_angular_acceleration=math.pi * (u.rad / (u.s * u.s)),
            theta_kP=1,
            x_kP=1,
            y_kP=1,
        )

        trajectory_config = wpimath.trajectory.TrajectoryConfig(maxVelocity=4.5, maxAcceleration=1)

        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            [
                Pose2d(0, 0, 0),  # Start at (0, 0)
                Pose2d(1, 0, 0),  # Move 1m forward
            ],
            trajectory_config,
        )

        return self.swerve.follow_trajectory_command(trajectory, follower_params, True)

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        # Reset the robot's heading reading to 0
        self.stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.stick.toggle_field_relative.onTrue(commands2.InstantCommand(self.teleop_command.toggle_field_relative))

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.stick.is_movement_commanded))
