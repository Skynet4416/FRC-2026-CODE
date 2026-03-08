// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RunBothIndexersCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;

  public RunBothIndexersCommand(
      SpindexerSubsystem spindexerSubsystem, ShooterIndexerSubsystem shooterIndexerSubsystem) {
    this.spindexerSubsystem = spindexerSubsystem;
    this.shooterIndexerSubsystem = shooterIndexerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexerSubsystem, shooterIndexerSubsystem);
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
    shooterIndexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Interrupt or cancel to end
  }
}
