// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class ShootCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;
  private final IntakeSubsystem leftIntake;
  private final IntakeSubsystem rightIntake;

  /** 
   * Runs the spindexer, shooter indexer, and intakes for shooting.
   */
  public ShootCommand(
      SpindexerSubsystem spindexerSubsystem,
      ShooterIndexerSubsystem shooterIndexerSubsystem,
      IntakeSubsystem leftIntake,
      IntakeSubsystem rightIntake) {
    this.spindexerSubsystem = spindexerSubsystem;
    this.shooterIndexerSubsystem = shooterIndexerSubsystem;
    this.leftIntake = leftIntake;
    this.rightIntake = rightIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexerSubsystem, shooterIndexerSubsystem, leftIntake, rightIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexerSubsystem.setPercentage(1.0);
    shooterIndexerSubsystem.setShooterIndexer(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexerSubsystem.setPercentage(1.0);
    shooterIndexerSubsystem.setShooterIndexer(1.0);

    // Intake runs at 0.2 when folded and shooting and at 1 when lowered
    if (leftIntake.isLowered()) {
      leftIntake.set(1.0);
    } else {
      leftIntake.set(0.2);
    }

    if (rightIntake.isLowered()) {
      rightIntake.set(1.0);
    } else {
      rightIntake.set(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
    shooterIndexerSubsystem.stop();
    leftIntake.stop();
    rightIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Will be interrupted by the trigger
  }
}
