package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(PhotonCamera camera) {
    this.camera = camera;
  }

  /**
   * Gets the latest pipeline result.
   */
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * Gets an estimated global pose from the camera.
   */

  public Optional<Pose2d> getEstimatedPose() {
    PhotonPipelineResult result = getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d cameraToTarget = target.getBestCameraToTarget();

      // Use known pose of AprilTag on the field (tag-to-field)
      Optional<Pose3d> tagPose = Constants.FieldConstants.kfieldLayout.getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) {
        return Optional.empty(); // No tag pose found for ID
      }

      Pose3d cameraPose = tagPose.get().transformBy(cameraToTarget.inverse()); // (camera-to-field)

      Pose2d estimatedPose2d = cameraPose.toPose2d();

      // Check if the pose is within field bounds
      if (isWithinFieldBounds(estimatedPose2d)) {
        return Optional.of(estimatedPose2d);
      }
    }
    return Optional.empty();
  }

  private boolean isWithinFieldBounds(Pose2d pose) {
    double fieldLength = Constants.FieldConstants.kfieldLayout.getFieldLength();
    double fieldWidth = Constants.FieldConstants.kfieldLayout.getFieldWidth();

    double x = pose.getX();
    double y = pose.getY();

    return x >= 0 && x <= fieldLength && y >= 0 && y <= fieldWidth;
  }

  @Override
  public void periodic() {
  }
}
