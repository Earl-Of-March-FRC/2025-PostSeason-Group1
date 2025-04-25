// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  //0 front left, 1 front right, 2 back left, 3 back right
  private MAXSwerveModule[] swerveModules = new MAXSwerveModule[4];
  private final SwerveDriveKinematics kinematics = DrivetrainConstants.kDriveKinematics;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() { 
    swerveModules[0] = new MAXSwerveModule(DrivetrainConstants.kFrontLeftDrivingMotorId, DrivetrainConstants.kFrontLeftTurningMotorId, DrivetrainConstants.kFrontLeftChassisAngularOffset);
    swerveModules[1] = new MAXSwerveModule(DrivetrainConstants.kFrontRightDrivingMotorId, DrivetrainConstants.kFrontRightTurningMotorId, DrivetrainConstants.kFrontRightChassisAngularOffset);
    swerveModules[2] = new MAXSwerveModule(DrivetrainConstants.kBackLeftDrivingMotorId, DrivetrainConstants.kBackLeftTurningMotorId, DrivetrainConstants.kBackLeftChassisAngularOffset);
    swerveModules[3] = new MAXSwerveModule(DrivetrainConstants.kBackRightDrivingMotorId, DrivetrainConstants.kBackRightTurningMotorId, DrivetrainConstants.kBackRightChassisAngularOffset);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  public void resetEncoders() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].resetEncoders();
    }
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  // m/s, m/s, rad/s
  public void driveVelocity(double xVelocity, double yVelocity, double angularVelocity) {
    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    driveChassisSpeeds(speeds);
  }

  public void drivePercent(double xPercent, double yPercent, double rotPercent) {
    double xVelocity = xPercent * DrivetrainConstants.kMaxSpeedMetersPerSecond;
    double yVelocity = yPercent * DrivetrainConstants.kMaxSpeedMetersPerSecond;
    double angularVelocity = rotPercent * DrivetrainConstants.kMaxAngularSpeed;
    driveVelocity(xVelocity, yVelocity, angularVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
