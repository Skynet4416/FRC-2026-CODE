package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedSubsystemIO {
  @AutoLog
  public static class LedIOInputs {
    public double batteryCharge = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LedIOInputs inputs) {}

  /** Sets the battery charge level (0.0 to 1.0). */
  public default void setBatteryCharge(double charge) {}
}
