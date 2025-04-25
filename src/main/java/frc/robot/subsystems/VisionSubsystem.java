package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
      Transform3d transform = target.getBestCameraToTarget();
      Pose2d pose = new Pose2d(transform.getTranslation().getX(), transform.getTranslation().getY(),
          transform.getRotation().toRotation2d());
      return Optional.of(pose);
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    // You can use this for logging or updating pose periodically
  }
}
