package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class VisionDriveToTagCmd extends Command {
  private final DrivetrainSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final double targetDistanceMeters = 1.0; // Stop 1 meter away
  private final double velocityMetersPerSecond = 0.4;
  private final PIDController turnController;

  public VisionDriveToTagCmd(DrivetrainSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);

    turnController = new PIDController(1.0, 0.0, 0.0);
    turnController.enableContinuousInput(-Math.PI, Math.PI); // For wrap-around at +- π

    SmartDashboard.putBoolean("At April Tag Distance", false);
  }

  @Override
  public void execute() {
    Optional<Pose2d> targetPoseOptional = visionSubsystem.getEstimatedPose();
    boolean atTargetDistance = false;

    if (targetPoseOptional.isPresent()) {
      Pose2d targetPose = targetPoseOptional.get();
      Translation2d tagTranslation = targetPose.getTranslation();
      double distanceToTag = tagTranslation.getNorm();
      atTargetDistance = (distanceToTag <= targetDistanceMeters);

      if (!atTargetDistance) {
        double angleToTagRad = Math.atan2(tagTranslation.getY(), tagTranslation.getX());
        double turnSpeed = turnController.calculate(angleToTagRad, 0.0);

        double directionX = tagTranslation.getX() / distanceToTag;
        double directionY = tagTranslation.getY() / distanceToTag;
        driveSubsystem.driveVelocity(
            velocityMetersPerSecond * directionX,
            velocityMetersPerSecond * directionY,
            turnSpeed);
      } else {
        driveSubsystem.driveVelocity(0.0, 0.0, 0.0);
      }
    } else {
      driveSubsystem.driveVelocity(0.0, 0.0, 0.0);
    }

    SmartDashboard.putBoolean("At Target Distance", atTargetDistance);
  }

  @Override
  public boolean isFinished() {
    return visionSubsystem.getEstimatedPose()
        .map(pose -> pose.getTranslation().getNorm() <= targetDistanceMeters)
        .orElse(false);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("At Target Distance",
        isFinished() && !interrupted);
    driveSubsystem.driveVelocity(0.0, 0.0, 0.0);
  }
}
