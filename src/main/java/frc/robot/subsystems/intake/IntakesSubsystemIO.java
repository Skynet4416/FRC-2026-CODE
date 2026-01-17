package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakesSubsystemIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public boolean lowered = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void setLowered(boolean lowered) {}
}
