package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants;

public class LedSubsystemIOCandle implements LedSubsystemIO {
  private final CANdle candle = new CANdle(Constants.Subsystems.Led.CANDLE_ID);

  public LedSubsystemIOCandle() {}

  @Override
  public void updateInputs(LedIOInputs inputs) {
    // Note: CANdle doesn't have many inputs to log for this simple case.
  }

  @Override
  public void setBatteryCharge(double charge) {
    // 0.0 charge = red, 1.0 charge = green
    int r = (int) ((1.0 - charge) * 255);
    int g = (int) (charge * 255);
    candle.setControl(new SolidColor(0, 64).withColor(new RGBWColor(r, g, 0)));
  }
}
