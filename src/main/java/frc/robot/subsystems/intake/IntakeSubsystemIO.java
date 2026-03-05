package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSubsystemIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double setpointRPM = 0.0;
    public boolean atSetpoint = false;
    public boolean lowered = false;
    public boolean connected = true;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void setLowered(boolean lowered) {}

  default void stop() {
    setVoltage(0);
  }
}
