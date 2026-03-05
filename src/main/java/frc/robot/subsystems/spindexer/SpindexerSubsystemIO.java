package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerSubsystemIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double setpointRPM = 0.0;
    public boolean atSetpoint = false;
    public boolean connected = true;
  }

  default void updateInputs(SpindexerIOInputs inputs) {}

  default void setTargetRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0);
  }

  default void setShooterIndexer(double percentage) {}
}
