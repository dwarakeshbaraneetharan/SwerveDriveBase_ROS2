# constants.py

from math import pi
#TODO update literally everything here
class ModuleConstants:
    kWheelDiameterMeters = 0.10033
    kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
    kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0)
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    kPTurning = 0.5

class DriveConstants:
    kTrackWidth = 0.686  # 27 inches to meters
    kWheelBase = 0.822  # 32.375 inches to meters
    kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * ModuleConstants.kDriveEncoderRot2Meter * pi
    kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / (0.686 / 2.0)
    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
    kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4

class OIConstants:
    kDeadband = 0.05
