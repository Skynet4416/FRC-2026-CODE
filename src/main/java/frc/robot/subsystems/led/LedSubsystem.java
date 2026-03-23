package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {
  private final LedSubsystemIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public LedSubsystem(LedSubsystemIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    double voltage = RobotController.getBatteryVoltage();
    double charge = (voltage - 8.0) / 4.0;
    charge = Math.max(0.0, Math.min(1.0, charge)); // Clamp to 0.0-1.0

    inputs.batteryCharge = charge;
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);

    io.setBatteryCharge(charge);
  }
}
