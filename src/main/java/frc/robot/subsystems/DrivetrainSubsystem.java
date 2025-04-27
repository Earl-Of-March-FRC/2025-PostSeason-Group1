// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveVelocity(double xVelocity, double yVelocity, double angularVelocity) {
    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    // driveChassisSpeeds(speeds);
  }
}
