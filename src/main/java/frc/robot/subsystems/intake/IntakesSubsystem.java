package frc.robot.subsystems.intake;

public class IntakesSubsystem {
  private final IntakesSubsystemIO io;
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakesSubsystem(IntakesSubsystemIO io) {
    this.io = io;
  }
}
