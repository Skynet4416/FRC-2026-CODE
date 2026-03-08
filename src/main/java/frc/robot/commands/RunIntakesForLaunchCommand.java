package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakesForLaunchCommand extends Command {
  private final IntakeSubsystem leftIntake;
  private final IntakeSubsystem rightIntake;

  public RunIntakesForLaunchCommand(IntakeSubsystem leftIntake, IntakeSubsystem rightIntake) {
    this.leftIntake = leftIntake;
    this.rightIntake = rightIntake;
    addRequirements(leftIntake, rightIntake);
  }

  @Override
  public void execute() {
    // If the intake is lowered, run it at full speed so we can still intake while launching.
    // If folded, run at 20% holding speed to help feed the ball up.
    leftIntake.setPercentage(leftIntake.isLowered() ? 1.0 : 0.2);
    rightIntake.setPercentage(rightIntake.isLowered() ? 1.0 : 0.2);
  }

  @Override
  public void end(boolean interrupted) {
    leftIntake.stop();
    rightIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs as long as the trigger is true
  }
}
