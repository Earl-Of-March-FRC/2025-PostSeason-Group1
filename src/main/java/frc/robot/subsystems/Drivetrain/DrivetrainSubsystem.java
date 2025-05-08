// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  //0 front left, 1 front right, 2 back left, 3 back right
  private final Module[] swerveModules = new Module[4];
  private final SwerveDriveKinematics kinematics = DrivetrainConstants.kDriveKinematics;
  private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem(ModuleIO[] moduleIOs) { 
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i] = new Module(moduleIOs[i], i);
    }
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].runSetpoint(states[i]);
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
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].periodic();
    }

    inputs.update(swerveModules);
    Logger.processInputs("Drive/Inputs", inputs);
  }

  @AutoLog
  public static class DrivetrainIOInputs {
    public SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    public SwerveModuleState[] desiredStates = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    public SwerveModuleState[] processedDesiredStates = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };


    public void update(Module[] swerveModules) {
      for (int i = 0; i < swerveModules.length; i++) {
        states[i] = swerveModules[i].getState();
        desiredStates[i] = swerveModules[i].getDesiredState();
        processedDesiredStates[i] = swerveModules[i].getProcessedDesiredState();
      }
    }
  }
};
