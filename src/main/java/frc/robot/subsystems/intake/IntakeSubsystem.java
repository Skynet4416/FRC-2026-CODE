package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeSubsystemIO io;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("Intake", 500.0);
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeSubsystemIO io) {
    this.io = io;
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public void setLowered() {
    io.setLowered(true);
  }

}
