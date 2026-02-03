package frc.robot.subsystems.drive.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelSubsystemIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setVoltage(double volts) {}
}
