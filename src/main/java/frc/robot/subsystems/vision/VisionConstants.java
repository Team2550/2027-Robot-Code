package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

public final class VisionConstants {
  public static final String kFrontCameraName = "FrontCamera";
  public static final String kLeftCameraName = "LeftCamera";

  public static final Transform3d kRobotToCamFront = new Transform3d(
      Units.inchesToMeters(0.0),
      Units.inchesToMeters(0.0),
      Units.inchesToMeters(0.0),
      new Rotation3d(0.0, 0.0, 0.0));

  public static final Transform3d kRobotToCamLeft = new Transform3d(
      Units.inchesToMeters(0.0),
      Units.inchesToMeters(0.0),
      Units.inchesToMeters(0.0),
      new Rotation3d(0.0, 0.0, 0.0));

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

}