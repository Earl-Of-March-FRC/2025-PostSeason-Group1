package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


import java.util.Optional;

public class VisionDriveToTagCmd extends Command {
  private final DrivetrainSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final double targetDistanceMeters = 1.0; // Stop 1 meter away
  private final double forwardSpeedMetersPerSecond = 0.4;
  private final PIDController turnController;

  public VisionDriveToTagCmd(DrivetrainSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);

    turnController = new PIDController(1.0, 0.0, 0.0);
    turnController.enableContinuousInput(-Math.PI, Math.PI); // For wrap-around at ±π
  }

  @Override
  public void execute() {
    Optional<Pose2d> targetPoseOptional = visionSubsystem.getEstimatedPose();

    if (targetPoseOptional.isPresent()) {
      Pose2d targetPose = targetPoseOptional.get();

      Translation2d tagTranslation = targetPose.getTranslation();
      double distanceToTag = tagTranslation.getNorm();

      double angleToTagRad = Math.atan2(tagTranslation.getY(), tagTranslation.getX()); // radians
      double turnSpeed = turnController.calculate(angleToTagRad, 0.0); // Target is straight ahead (0 rad)

      if (distanceToTag > targetDistanceMeters) {
        driveSubsystem.drive(forwardSpeedMetersPerSecond, 0.0, turnSpeed);
      } else {
        driveSubsystem.drive(0.0, 0.0, 0.0);
      }

    } else {
      driveSubsystem.drive(0.0, 0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return visionSubsystem.getEstimatedPose()
        .map(pose -> pose.getTranslation().getNorm() <= targetDistanceMeters)
        .orElse(false);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0);
  }
}
