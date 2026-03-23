package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

public class ledSubsystemIOCandle implements ledSubsystemIO {
  private final CANdle candle;

  public ledSubsystemIOCandle() {
    candle = new CANdle(6);
  }

  @Override
  public void setAnimation(ControlRequest request) {
    candle.clearAllAnimations();
    candle.setControl(request);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {}
}
