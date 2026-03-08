package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakesForLaunchCommand extends Command {
  private final IntakeSubsystem leftIntake;
  private final IntakeSubsystem rightIntake;

  public RunIntakesForLaunchCommand(IntakeSubsystem leftIntake, IntakeSubsystem rightIntake) {
    this.leftIntake = leftIntake;
    this.rightIntake = rightIntake;
    // We intentionally do not require the intake subsystems here so we don't
    // interrupt any state-transition commands (like toggling).
  }

  @Override
  public void execute() {
    if (!leftIntake.isLowered()) {
      leftIntake.setPercentage(0.2);
    }
    if (!rightIntake.isLowered()) {
      rightIntake.setPercentage(0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!leftIntake.isLowered()) {
      leftIntake.stop();
    }
    if (!rightIntake.isLowered()) {
      rightIntake.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Runs as long as the trigger is true
  }
}
