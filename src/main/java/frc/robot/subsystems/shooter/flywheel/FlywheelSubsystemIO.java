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
    public boolean atSetpoint = false;
    public boolean connected = true;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setTargetRADS(double radiansPerSecond) {}

  default void setVoltage(double volts) {}

  default void configPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void stop() {
    setVoltage(0);
  }
}
