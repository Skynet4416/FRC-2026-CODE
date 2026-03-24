// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import org.littletonrobotics.junction.Logger;

public class RunBothIndexersCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;
  private double runTime = 0.0;
  private double stuckTime = 0.0;
  private boolean direction = true;
  private final double targetPercentage;

  private final double FLIP_INTERVAL = 6.0;
  private final double FLIP_DURATION = 0.2;

  public RunBothIndexersCommand(
      SpindexerSubsystem spindexerSubsystem,
      ShooterIndexerSubsystem shooterIndexerSubsystem,
      double targetPercentage) {
    this.spindexerSubsystem = spindexerSubsystem;
    this.shooterIndexerSubsystem = shooterIndexerSubsystem;
    this.targetPercentage = targetPercentage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexerSubsystem, shooterIndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexerSubsystem.setPercentage(targetPercentage);
    shooterIndexerSubsystem.setShooterIndexer(targetPercentage);
    runTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexerSubsystem.setPercentage(true ? targetPercentage : -targetPercentage);
    shooterIndexerSubsystem.setShooterIndexer(targetPercentage);

    Logger.recordOutput("Indexer direction?", direction);
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - runTime > (direction ? FLIP_INTERVAL : FLIP_DURATION)) {
      direction = !direction;
      runTime = Timer.getFPGATimestamp();
    }
    if (Math.abs(spindexerSubsystem.getVelocityRPM()) > 500) {
      stuckTime = currentTime;
    } else if (currentTime - stuckTime > 0.5) {
      // direction = false;
      stuckTime = currentTime;
    }
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
