package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class TrajCommnd extends Command {

  private Command followCommand;
  private final double[] arr;
  private final Drive driveSubsystem;

  public TrajCommnd(AutoFactory autoFactory, String pathName, Drive drive) {
    followCommand = autoFactory.trajectoryCmd(pathName);
    Pose2d[] poses = Choreo.loadTrajectory(pathName).get().getPoses();
    arr = new double[poses.length * 3];
    int ndx = 0;
    for (Pose2d pose : poses) {
      Translation2d translation = AllianceFlipUtil.apply(pose.getTranslation());
      arr[ndx + 0] = translation.getX();
      arr[ndx + 1] = translation.getY();
      arr[ndx + 2] = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
      ndx += 3;
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
