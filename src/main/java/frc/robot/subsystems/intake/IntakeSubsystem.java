package frc.robot.subsystems.intake;

public class IntakeSubsystem {
  private final IntakeSubsystemIO io;
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeSubsystemIO io) {
    this.io = io;
  }
}
