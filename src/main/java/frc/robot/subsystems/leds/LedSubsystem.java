package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.fasterxml.jackson.databind.deser.SettableAnyProperty;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private final ledSubsystemIO io;
  protected final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private int ledMode = -1;
  private double setTime = 0;

  public LedSubsystem(ledSubsystemIO io) {
    this.io = io;
    SetIdle();
  }

  private boolean SetValue(int val) {
    if(val != 0)
      setTime = Timer.getFPGATimestamp();
    if(ledMode == val)
    return false;
    ledMode = val;
    return true;
  }

  public void SetIdle() {
    if (SetValue(0)) return;
    ControlRequest ledRequest;
    ledRequest = new SolidColor(0, 64).withColor(new RGBWColor(255, 120, 0));
    this.io.setAnimation(ledRequest);
  }

  public void SetAiming() {
    if (SetValue(1)) return;
    ControlRequest ledRequest;
    ledRequest = new SolidColor(0, 64).withColor(new RGBWColor(0, 255, 0));
    this.io.setAnimation(ledRequest);
  }

  public void SetShooting() {
    if (SetValue(2)) return;
    ControlRequest ledRequest;
    ledRequest = new StrobeAnimation(0, 64).withColor(new RGBWColor(0, 255, 0)).withFrameRate(6);
    this.io.setAnimation(ledRequest);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if(Timer.getFPGATimestamp() - setTime > 0.2) {
      SetIdle();
    }
  }
}
