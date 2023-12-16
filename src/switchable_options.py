import math
from functools import cache
from pathlib import Path

import ctre
import rev
from swervepy import u
from swervepy.impl import Falcon500CoaxialDriveComponent, Falcon500CoaxialAzimuthComponent, PigeonGyro, \
    CoaxialSwerveModule, AbsoluteCANCoder, NEOCoaxialDriveComponent, NEOCoaxialAzimuthComponent
from swervepy.impl.sensor import Pigeon2Gyro, SparkMaxEncoderType
from wpimath.geometry import Translation2d, Rotation2d


def comp_2023():
    class Inner:
        TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
        WHEEL_BASE = (24.75 * u.inch).m_as(u.m)
        MAX_VELOCITY = 4 * (u.m / u.s)
        MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

        DRIVE_PARAMS = Falcon500CoaxialDriveComponent.Parameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            peak_current_duration=0.01,
            neutral_mode=ctre.NeutralMode.Coast,
            kP=0.1,
            kI=0,
            kD=0,
            kS=0.16954 / 12,
            kV=2.1535 / 12,
            kA=0.27464 / 12,
            invert_motor=False,
        )
        AZIMUTH_PARAMS = Falcon500CoaxialAzimuthComponent.Parameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            peak_current_duration=0.01,
            neutral_mode=ctre.NeutralMode.Brake,
            kP=0.3,
            kI=0,
            kD=0,
            invert_motor=True,
        )

        GYRO = PigeonGyro(0, False)

        MODULES = (
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(4, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(3, Rotation2d.fromDegrees(331.435), AZIMUTH_PARAMS, AbsoluteCANCoder(0)),
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(1, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(6, Rotation2d.fromDegrees(156.093), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(7, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(2, Rotation2d.fromDegrees(45.263), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ),
            CoaxialSwerveModule(
                Falcon500CoaxialDriveComponent(5, DRIVE_PARAMS),
                Falcon500CoaxialAzimuthComponent(0, Rotation2d.fromDegrees(211.201), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
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

        DRIVE_PARAMS = NEOCoaxialDriveComponent.Parameters(
            wheel_circumference=4 * math.pi * u.inch,
            gear_ratio=6.75 / 1,  # SDS Mk4i L2
            max_speed=MAX_VELOCITY,
            open_loop_ramp_rate=0.25,
            closed_loop_ramp_rate=0,
            continuous_current_limit=40,
            peak_current_limit=60,
            neutral_mode=rev.CANSparkMax.IdleMode.kCoast,
            kP=0.1,  # TODO: Characterize drivetrain
            kI=0,
            kD=0,
            kS=0,
            kV=0,
            kA=0,
            invert_motor=False,
        )
        AZIMUTH_PARAMS = NEOCoaxialAzimuthComponent.Parameters(
            gear_ratio=150 / 7,  # SDS Mk4i
            max_angular_velocity=MAX_ANGULAR_VELOCITY,
            ramp_rate=0,
            continuous_current_limit=25,
            peak_current_limit=40,
            neutral_mode=rev.CANSparkMax.IdleMode.kBrake,
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


# Dictionary of robot ID values possible in the ROBOT_ID file. Each key's value is a function that returns a class of
# constants
OPTIONS = {
    "0": comp_2023,
    "1": dev_2024
}


@cache
def get_robot_specific_options():
    # Load robot identifier from file
    try:
        with open(Path(__file__).resolve().parent / "ROBOT_ID", "r") as f:
            tag = f.read().strip()
    except OSError:
        raise Exception("ROBOT_ID does not exist or is malformed. Use deploy script with an argument to set ROBOT_ID.")

    # Pick options from robot ID
    try:
        options = OPTIONS[tag]
    except KeyError:
        raise Exception(f"Robot ID {tag} in deploy command does not have a matching option set in code.")

    return options()
