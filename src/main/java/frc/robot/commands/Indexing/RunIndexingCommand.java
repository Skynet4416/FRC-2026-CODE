package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RunIndexingCommand extends Command {
  private final SpindexerSubsystem m_spindexer;
  private final ShooterIndexerSubsystem m_shooterIndexer;
  private final IntakeSubsystem m_intakeLeft;
  private final IntakeSubsystem m_intakeRight;
  double targetSpeed = 0.1;

  public RunIndexingCommand(
      SpindexerSubsystem spindexer,
      ShooterIndexerSubsystem shooterIndexer,
      IntakeSubsystem intakeLeft,
      IntakeSubsystem intakeRight,
      double speed) {
    addRequirements(spindexer, shooterIndexer);
    m_spindexer = spindexer;
    m_shooterIndexer = shooterIndexer;
    m_intakeLeft = intakeLeft;
    m_intakeRight = intakeRight;
    targetSpeed = speed;
  }

  @Override
  public void initialize() {
    m_spindexer.set(targetSpeed);
    m_shooterIndexer.set(targetSpeed);
    if (!m_intakeLeft.isLowered()) m_intakeLeft.set(0.3);
    if (!m_intakeRight.isLowered()) m_intakeRight.set(0.3);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_spindexer.set(0);
    m_shooterIndexer.set(0);
    if (!m_intakeLeft.isLowered()) m_intakeLeft.set(0.0);
    if (!m_intakeRight.isLowered()) m_intakeRight.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
