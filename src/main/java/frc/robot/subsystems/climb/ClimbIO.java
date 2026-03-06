package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    // NEO
    public double neoPositionRads = 0.0;
    public double neoVelocityRadsPerSec = 0.0;
    public double neoAppliedVolts = 0.0;
    public double neoCurrentAmps = 0.0;

    // Kraken
    public double krakenPositionRads = 0.0;
    public double krakenVelocityRadsPerSec = 0.0;
    public double krakenAppliedVolts = 0.0;
    public double krakenCurrentAmps = 0.0;

    // Pneumatics
    public boolean lowered = false;

    public boolean neoLeaderConnected = true;
    public boolean neoFollowerConnected = true;
    public boolean krakenConnected = true;
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the targeted position for the NEOs in radians. */
  default void setNeoAngle(double rads) {}

  /** Set the targeted position for the Kraken in radians. */
  default void setKrakenAngle(double rads) {}

  /** Sets whether the climber pneumatics are lowered. */
  default void setLowered(boolean lowered) {}

  default void stop() {}
}
