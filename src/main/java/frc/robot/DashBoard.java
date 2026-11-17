package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DashBoard extends SubsystemBase {
  private final Field2d field = new Field2d();

  private DashBoard() {
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // ose2d robotPose = CommandSwerveDrivetrain.
    // field.setRobotPose(robotPose);
  }
}