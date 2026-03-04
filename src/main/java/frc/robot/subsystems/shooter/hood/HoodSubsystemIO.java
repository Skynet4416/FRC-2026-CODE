package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodSubsystemIO {
  @AutoLog
  public static class HoodIOInputs {
    public double angle = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double targetAngle = 0.0;
    public boolean connected = true;
    public boolean atSetpoint = false;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  default void setVoltage(double volts) {}

  /** Run the motor at the specified percent output. */
  default void set(double percentOutput) {}

  /** Run to the specified angle in degrees. */
  default void setTargetAngle(double degrees) {}

  default void setTargetAngleWithVelocity(double degrees, double velocityRPM) {}

  /** Stop the motor. */
  default void stop() {}

  /** Reset the position of the encoder to the specified angle in degrees. */
  default void setAngle(double degrees) {}
}
