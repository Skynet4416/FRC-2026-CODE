package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeSubsystemIO io;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("RollerRPM", 500.0);
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeSubsystemIO io) {
    this.io = io;
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public void setLowered(boolean lowered) {
    io.setLowered(lowered);
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

  public boolean isLowered() {
    return inputs.lowered;
  }

  public void Stop() {
    setTargetRPM(0);
  }

  public Command runRollerCommand() {
    return Commands.runEnd(() -> setTargetRPM(targetRpm.get()), () -> setTargetRPM(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RollerRPM", inputs);
  }
}
