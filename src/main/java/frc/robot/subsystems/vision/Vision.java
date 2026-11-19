package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.math.geometry.Pose2d;

public class Vision extends SubsystemBase {
  PhotonCamera Camera0 = new PhotonCamera(VisionConstants.kFrontCameraName);
  PhotonCamera Camera1 = new PhotonCamera(VisionConstants.kLeftCameraName);
  PhotonPoseEstimator photonEstimator0 = new PhotonPoseEstimator(FieldConstants.kTagLayout,
      VisionConstants.kRobotToCamFront);
  PhotonPoseEstimator photonEstimator1 = new PhotonPoseEstimator(FieldConstants.kTagLayout,
      VisionConstants.kRobotToCamLeft);
  Matrix<N3, N1> kSingleTagStdDevs = VisionConstants.kSingleTagStdDevs;
  Matrix<N3, N1> kMultiTagStdDevs = VisionConstants.kMultiTagStdDevs;
  private final Alert camera0Alert = new Alert("Camera 0 is not connected", AlertType.kWarning);
  private final Alert camera1Alert = new Alert("Camera 1 is not connected", AlertType.kWarning);
  private final CommandSwerveDrivetrain drivetrain;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    camera0Alert.set(!Camera0.isConnected());
    camera1Alert.set(!Camera1.isConnected());

    // Process both cameras independently
    processCamera(Camera0, photonEstimator0);
    processCamera(Camera1, photonEstimator1);
  }

  private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
    for (var result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = estimator.estimateLowestAmbiguityPose(result);
      }

      // Calculate std devs specifically for this result/camera
      Matrix<N3, N1> estStdDevs = calculateEstimationStdDevs(visionEst, result.getTargets(), estimator);

      visionEst.ifPresent(
          est -> {
            Pose2d pose = est.estimatedPose.toPose2d();
            double timestamp = est.timestampSeconds;
            drivetrain.addVisionMeasurement(pose, timestamp, estStdDevs);
          });
    }
  }

  private Matrix<N3, N1> calculateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator photonEstimator) {
    if (estimatedPose.isEmpty()) {
      return kSingleTagStdDevs;
    }

    var estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return kSingleTagStdDevs;
    }

    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }

}