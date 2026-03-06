package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerSubsystemIO io;
  private final SysIdRoutine sysIdRoutine;
  protected final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("IndexRPM", 500.0);
  private final ShooterIndexerSubsystem shooterIndexer;

  public SpindexerSubsystem(
      SpindexerSubsystemIO io,
      frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerIO shooterIndexerIO) {
    this.io = io;
    this.shooterIndexer = new ShooterIndexerSubsystem(shooterIndexerIO);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Spindexer/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public void setShooterIndexer(double percentage) {
    shooterIndexer.setShooterIndexer(percentage);
  }

  public void stop() {
    shooterIndexer.setShooterIndexer(0);
  }

  public Command runIndexerCommand() {
    return Commands.run(() -> io.set(1.0));
  }

  public Command runShooterIndexerCommand() {
    return Commands.run(() -> setShooterIndexer(1.0));
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void stopShooterIndexer() {
    shooterIndexer.stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }
}
