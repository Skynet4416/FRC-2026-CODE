package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

/**
 * A calibration command that sets the flywheel RPM and hood angle to given values and shoots.
 * Prints the launcherToTargetDistance, RPM, and hood angle to the console for calibration.
 */
public class TestShootCommand extends Command {
  private final FlywheelSubsystem flywheel;
  private final HoodSubsystem hood;
  private final ShooterIndexerSubsystem shooterIndexer;
  private final SpindexerSubsystem spindexer;
  private final double targetRPM;
  private final double targetHoodAngleDeg;
  private boolean hasPrintedShot = false;

  /**
   * Creates a new TestShootCommand.
   *
   * @param flywheel The flywheel subsystem
   * @param hood The hood subsystem
   * @param shooterIndexer The shooter indexer subsystem
   * @param spindexer The spindexer subsystem
   * @param targetRPM The desired flywheel speed in RPM
   * @param targetHoodAngleDeg The desired hood angle in degrees
   */
  public TestShootCommand(
      FlywheelSubsystem flywheel,
      HoodSubsystem hood,
      ShooterIndexerSubsystem shooterIndexer,
      SpindexerSubsystem spindexer,
      double targetRPM,
      double targetHoodAngleDeg) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.shooterIndexer = shooterIndexer;
    this.spindexer = spindexer;
    this.targetRPM = targetRPM;
    this.targetHoodAngleDeg = targetHoodAngleDeg;

    addRequirements(flywheel, hood, shooterIndexer, spindexer);
  }

  @Override
  public void initialize() {
    hasPrintedShot = false;
    var params = LaunchCalculator.getInstance().getParameters();
    double launcherToTargetDistance = params.distanceNoLookahead();
    System.out.println(
        "[TestShoot] START | distance="
            + String.format("%.2f", launcherToTargetDistance)
            + "m | RPM="
            + String.format("%.1f", targetRPM)
            + " | hoodAngle="
            + String.format("%.1f", targetHoodAngleDeg)
            + "deg");
  }

  @Override
  public void execute() {
    // Set flywheel and hood to the desired values
    flywheel.setTargetRPM(targetRPM);
    hood.setTargetAngle(targetHoodAngleDeg);

    // Feed the ball once flywheel and hood are ready
    if (flywheel.atSetpoint() && hood.atSetpoint()) {
      shooterIndexer.setShooterIndexer(1.0);
      spindexer.setPercentage(1.0);
      if (!hasPrintedShot) {
        System.out.println("[TestShoot] SHOOTING! Flywheel & hood at setpoint.");
        hasPrintedShot = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    hood.stop();
    shooterIndexer.stop();
    spindexer.stop();
    System.out.println("[TestShoot] END (interrupted=" + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
