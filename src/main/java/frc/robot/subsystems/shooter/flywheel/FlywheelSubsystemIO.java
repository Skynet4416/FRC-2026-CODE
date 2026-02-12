package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelSubsystemIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double setpointRPM = 0.0;
    public boolean connected = true;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setVoltage(double volts) {}
}
