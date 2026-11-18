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
  private Matrix<N3, N1> curStdDevs;
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
    getCameraData();
  }

  void getCameraData() {
    PhotonCamera cameraName = (Camera0.getLatestResult().getTargets().size() > Camera1.getLatestResult().getTargets()
        .size()) ? Camera0 : Camera1;
    PhotonPoseEstimator photonEstimator = (cameraName == Camera0) ? photonEstimator0 : photonEstimator1;

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : cameraName.getAllUnreadResults()) {
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEst, result.getTargets(), photonEstimator);

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();
            Pose2d pose = est.estimatedPose.toPose2d();
            Double timestamp = est.timestampSeconds;
            drivetrain.addVisionMeasurement(pose, timestamp, estStdDevs);
          });
    }

  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator photonEstimator) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
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
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

}