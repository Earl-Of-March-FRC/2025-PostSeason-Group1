// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionDriveToTagCMD extends Command {
  private final DrivetrainSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double targetDistanceMeters = 1.0; // Stop 1 meter away
  private final double speedMetersPerSecond = 0.5;

  /** Creates a new VisionDriveToTagCMD. */
  public VisionDriveToTagCMD(DrivetrainSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(this.driveSubsystem, this.visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Pose2d> targetPoseOptional = visionSubsystem.getEstimatedPose();

    if (targetPoseOptional.isPresent()) {
      Pose2d targetPose = targetPoseOptional.get();
      double distance = targetPose.getTranslation().getNorm();

      if (distance > targetDistanceMeters) {
        // Drive forward toward the tag (assumes tag is directly ahead of robot)
        driveSubsystem.drive(speedMetersPerSecond, 0.0, 0.0);
      } else {
        // Stop when within 1 meter
        driveSubsystem.drive(0.0, 0.0, 0.0);
      }
    } else {
      // No tag found, stop
      driveSubsystem.drive(0.0, 0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
