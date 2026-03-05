package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerSubsystemIO io;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("IndexRPM", 500.0);
  protected final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerSubsystemIO io) {
    this.io = io;
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }
}
