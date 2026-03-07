package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {
  private final SpindexerSubsystemIO io;
  protected final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerSubsystemIO io) {
    this.io = io;
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

  public void set(double percentage) {
    io.set(percentage);
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.set(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }
}
