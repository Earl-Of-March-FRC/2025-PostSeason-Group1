// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain.MAXSwerveModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DrivetrainConstants {
    //TODO fix ids
    public static final int kFrontLeftDrivingMotorId = 1;
    public static final int kFrontLeftTurningMotorId = 2;
    public static final int kFrontRightDrivingMotorId = 3;
    public static final int kFrontRightTurningMotorId = 4;
    public static final int kBackLeftDrivingMotorId = 5;
    public static final int kBackLeftTurningMotorId = 6;
    public static final int kBackRightDrivingMotorId = 7;
    public static final int kBackRightTurningMotorId = 8;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; // Default 4.8 - Max net robot translational speed
    public static final double kMaxWheelSpeedMetersPerSecond = 4.8; // Max possible speed for wheel
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kBalleyPopMetersPerSecond = 0.8; // Max net robot translational speed when intaking algae
                                                                // stacked on coral
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAccelerationMetersPerSecondSquaredPathfinding = 1;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // The chassis angular offset is the angle between the chassis and the module
    // zero position. This is used to correct the module angle to be relative to the
    // chassis.
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kTrackWidthMeters = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBaseMeters = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
        new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
        new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
        new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2));

    public static final ModuleIO[] kModuleIOsReal = {
        new MAXSwerveModuleIO(kFrontLeftDrivingMotorId, kFrontLeftTurningMotorId, kFrontLeftChassisAngularOffset),
        new MAXSwerveModuleIO(kFrontRightDrivingMotorId, kFrontRightTurningMotorId, kFrontRightChassisAngularOffset),
        new MAXSwerveModuleIO(kBackLeftDrivingMotorId, kBackLeftTurningMotorId, kBackLeftChassisAngularOffset),
        new MAXSwerveModuleIO(kBackRightDrivingMotorId, kBackRightTurningMotorId, kBackRightChassisAngularOffset)
    };

  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
