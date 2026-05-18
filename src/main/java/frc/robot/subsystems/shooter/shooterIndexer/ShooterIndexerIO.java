package frc.robot.subsystems.shooter.shooterIndexer;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIndexerIO {
  @AutoLog
  public static class ShooterIndexerIOInputs {
    public double appliedVolts = 0.0;
    public double requestedPercentage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double velocityRPM = 0.0;
    public boolean connected = true;
  }

  default void updateInputs(ShooterIndexerIOInputs inputs) {}

  default void setPercentage(double percentage) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0);
  }
}
