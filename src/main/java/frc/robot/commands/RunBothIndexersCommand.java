// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class RunBothIndexersCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;
  private double runTime = 0.0;
  private double stuckTime = 0.0;

  private double ballTime = 0.0;
  private double reverseTime = 0.0;
  private final double EMPTY_DURATION = 0.6;
  private final double COOLDOWN_DURATION = 1.5;

  private boolean direction = true;
  private double targetPercentage;

  private final double FLIP_INTERVAL = 1.5;
  private final double FLIP_DURATION = 0.2;

  private final LoggedTunableNumber tunePercentage = new LoggedTunableNumber("IndexerSpeed", 1.0);

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

    ballTime = Timer.getFPGATimestamp() + EMPTY_DURATION * 2.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexerSubsystem.setPercentage(direction ? tunePercentage.get() : -tunePercentage.get());
    shooterIndexerSubsystem.setShooterIndexer(direction ? targetPercentage : -targetPercentage);

    Logger.recordOutput("Indexer direction?", direction);
    double currentTime = Timer.getFPGATimestamp();

    if (shooterIndexerSubsystem.getCurrentAmps() > 20) {
      ballTime = currentTime;
    }

    if (currentTime - ballTime > (direction ? EMPTY_DURATION : FLIP_DURATION)) {
      if (direction && currentTime - reverseTime < COOLDOWN_DURATION) return;
      direction = !direction;
      reverseTime = Timer.getFPGATimestamp();
    }

    // if (Math.abs(spindexerSubsystem.getVelocityRPM()) > 500) {
    //   stuckTime = currentTime;
    // } else if (currentTime - stuckTime > 0.5) {
    //   direction = false;
    //   stuckTime = currentTime;
    // }
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
