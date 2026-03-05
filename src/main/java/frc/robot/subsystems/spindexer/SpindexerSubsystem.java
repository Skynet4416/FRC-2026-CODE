package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerSubsystemIO io;
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("IndexRPM", 500.0);
  protected final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerSubsystemIO io) {
    this.io = io;
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

  public void stop() {
    io.stop();
  }

  public Command runIndexerCommand() {
    return Commands.run(() -> setTargetRPM(targetRpm.get()));
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
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
