package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface ledSubsystemIO {
  @AutoLog
  public static class LedIOInputs {
    public int ledMode;
  }

  default void updateInputs(LedIOInputs inputs) {}

  default void setAnimation(ControlRequest request) {}
}
