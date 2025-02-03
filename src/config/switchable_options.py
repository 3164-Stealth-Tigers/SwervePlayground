import copy
import math
from functools import cache
from pathlib import Path

from swervepy import u
from swervepy.impl import (
    Falcon500CoaxialDriveComponent,
    Falcon500CoaxialAzimuthComponent,
    PigeonGyro,
    CoaxialSwerveModule,
    AbsoluteCANCoder,
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    TypicalDriveComponentParameters,
    TypicalAzimuthComponentParameters,
    NeutralMode,
)
from swervepy.impl.motor import DummyCoaxialDriveComponent, DummyCoaxialAzimuthComponent
from swervepy.impl.sensor import Pigeon2Gyro, DummyGyro
from wpimath.geometry import Translation2d, Rotation2d


def comp_2023():
    class Inner:
        TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (24.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

        DRIVE_PARAMS = TypicalDriveComponentParameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.COAST,
            kP=0.1,
            kI=0,
            kD=0,
            kS=0.16954 / 12,
            kV=2.1535 / 12,
            kA=0.27464 / 12,
            invert_motor=False,
        )
        AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.BRAKE,
            kP=0.3,
            kI=0,
            kD=0,
            invert_motor=True,
        )

        GYRO = PigeonGyro(0, False)

        MODULES = (
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(4, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(3, Rotation2d.fromDegrees(0), AZIMUTH_PARAMS, AbsoluteCANCoder(0)),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(1, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(6, Rotation2d.fromDegrees(0), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(7, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(2, Rotation2d.fromDegrees(0), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(5, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(0, Rotation2d.fromDegrees(0), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
        )

    return Inner()


def dev_2024():
    class Inner:
        TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (24.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)  # TODO: Measure
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)  # TODO: Measure

        DRIVE_PARAMS = TypicalDriveComponentParameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.COAST,
            kP=0.1,  # TODO: Characterize drivetrain
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=False,
        )
        AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.BRAKE,
            kP=0.01,
            kI=0,
            kD=0,
            invert_motor=True,
        )

        GYRO = Pigeon2Gyro(0, False)

        MODULES = (
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(1, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(2, Rotation2d.fromDegrees(107.226562), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(3, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(4, Rotation2d.fromDegrees(160.136719), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(5, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(6, Rotation2d.fromDegrees(307.089844), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(7, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(8, Rotation2d.fromDegrees(355.693359), AZIMUTH_PARAMS, AbsoluteCANCoder(4)),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
        )

    return Inner()


def demo_1():
    class Inner:
        TRACK_WIDTH = (18.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (18.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

        DRIVE_PARAMS = TypicalDriveComponentParameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.COAST,
            kP=0.1,
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=True,
        )
        AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.BRAKE,
            kP=0.01,
            kI=0,
            kD=0,
            invert_motor=True,
        )

        GYRO = DummyGyro(0, False)

        MODULES = [
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(1, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(2, Rotation2d.fromDegrees(224.208984), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(3, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(4, Rotation2d.fromDegrees(72.861328), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(5, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(6, Rotation2d.fromDegrees(97.294922), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(7, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(8, Rotation2d.fromDegrees(312.451172), AZIMUTH_PARAMS, AbsoluteCANCoder(4)),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
        ]

    return Inner()


def demo_2():
    class Inner:
        TRACK_WIDTH = (18.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (18.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

        DRIVE_PARAMS = TypicalDriveComponentParameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.COAST,
            kP=0.1,
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=True,
        )
        NEO_AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.BRAKE,
            kP=0.01,
            kI=0,
            kD=0,
            invert_motor=True,
        )
        FALCON_AZIMUTH_PARAMS = TypicalAzimuthComponentParameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=NeutralMode.BRAKE,
            kP=0.3,
            kI=0,
            kD=0,
            invert_motor=True,
        )

        GYRO = Pigeon2Gyro(0, False)

        MODULES = [
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(1, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(
                    2, Rotation2d.fromDegrees(314.6484375), NEO_AZIMUTH_PARAMS, AbsoluteCANCoder(1)
                ),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(3, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(
                    2, Rotation2d.fromDegrees(62.666015625), FALCON_AZIMUTH_PARAMS, AbsoluteCANCoder(2)
                ),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(5, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(
                    6, Rotation2d.fromDegrees(151.5234375), NEO_AZIMUTH_PARAMS, AbsoluteCANCoder(3)
                ),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(7, DRIVE_PARAMS),
                NEOCoaxialAzimuthComponent(
                    8, Rotation2d.fromDegrees(34.98046875), NEO_AZIMUTH_PARAMS, AbsoluteCANCoder(4)
                ),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
        ]

    return Inner()


def dummy():
    class Inner:
        TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (24.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

        GYRO = DummyGyro()

        MODULES = (
            CoaxialSwerveModule(
                DummyCoaxialDriveComponent(),
                DummyCoaxialAzimuthComponent(),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                DummyCoaxialDriveComponent(),
                DummyCoaxialAzimuthComponent(),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                DummyCoaxialDriveComponent(),
                DummyCoaxialAzimuthComponent(),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                DummyCoaxialDriveComponent(),
                DummyCoaxialAzimuthComponent(),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
        )

    return Inner()


# Dictionary of robot ID values possible in the ROBOT_ID file. Each key's value is a function that returns a class of
# constants
OPTIONS = {
    "0": comp_2023,
    "1": dev_2024,
    "2": demo_1,
    "3": demo_2,
    "4": dummy,
}


@cache
def get_robot_specific_options():
    # Load robot identifier from file
    try:
        with open(Path(__file__).resolve().parent.parent / "ROBOT_ID", "r") as f:
            tag = f.read().strip()
    except OSError:
        raise Exception("ROBOT_ID does not exist or is malformed. Use deploy script with an argument to set ROBOT_ID.")

    # Pick options from robot ID
    try:
        options = OPTIONS[tag]
    except KeyError:
        raise Exception(f"Robot ID {tag} in deploy command does not have a matching option set in code.")

    return options()
