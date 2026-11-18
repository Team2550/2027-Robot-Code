package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DashBoard extends SubsystemBase {
  private final Field2d field = new Field2d();
  private final CommandSwerveDrivetrain drivetrain;

  public DashBoard(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putData("Field", field);
    System.out.println("DashBoard2");
  }

  @Override
  public void periodic() {
    Pose2d robotPose = drivetrain.getState().Pose;
    field.setRobotPose(robotPose);
  }
}