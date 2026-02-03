package frc.robot.subsystems.drive.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelSubsystemIO io;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("FlywheelRPM", 4500.0);

  protected final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelSubsystemIO io) {
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

  public double getTempCelsius() {
    return inputs.tempCelsius;
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command runFlywheelCommand() {
    return Commands.runEnd(() -> setTargetRPM(targetRpm.get()), () -> stop(), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
