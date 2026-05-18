package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class TrajCommnd extends Command {
  private final Command followCommand;
  private final double[] arr;
  private final Drive driveSubsystem;

  // Change: Pass the AutoTrajectory instead of the AutoFactory
  public TrajCommnd(AutoTrajectory trajectory, String pathName, Drive drive) {
    // 1. This .cmd() is what makes .bind() work!
    this.followCommand = trajectory.cmd();

    // 2. Keep your logging logic
    var trajOpt = Choreo.loadTrajectory(pathName);
    if (trajOpt.isPresent()) {
      Pose2d[] poses = trajOpt.get().getPoses();
      arr = new double[poses.length * 3];
      int ndx = 0;
      for (Pose2d pose : poses) {
        Translation2d translation = AllianceFlipUtil.apply(pose.getTranslation());
        arr[ndx + 0] = translation.getX();
        arr[ndx + 1] = translation.getY();
        arr[ndx + 2] = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
        ndx += 3;
      }
    } else {
      arr = new double[0];
    }

    this.driveSubsystem = drive;
    addRequirements(followCommand.getRequirements());
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Choreo/Trajectory", arr);
    followCommand.initialize();
  }

  @Override
  public void execute() {
    followCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    followCommand.end(interrupted);
    driveSubsystem.stopWithX();
  }
}
