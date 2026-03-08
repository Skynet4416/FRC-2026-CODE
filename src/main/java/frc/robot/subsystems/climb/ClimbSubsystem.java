package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void setNeoAngle(double rads) {
    io.setNeoAngle(rads);
  }

  public void setNeo(double percentage) {
    io.setNeo(percentage);
  }

  public void setKrakenAngle(double rads) {
    io.setKrakenAngle(rads);
  }

  public void setLowered(boolean lowered) {
    io.setLowered(lowered);
  }

  public boolean isLowered() {
    return inputs.lowered;
  }

  public void stop() {
    io.stop();
  }

  public Command setLoweredCommand(boolean lowered) {
    return Commands.runOnce(() -> setLowered(lowered), this);
  }
}
