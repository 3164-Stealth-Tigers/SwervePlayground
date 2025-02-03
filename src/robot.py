from typing import Optional

import commands2
import wpilib

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None
        self.test_command: Optional[commands2.Command] = None

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def testInit(self) -> None:
        self.test_command = self.container.get_test_command()
        if self.test_command:
            self.test_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()
        if self.test_command:
            self.test_command.cancel()
