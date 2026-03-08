package frc.robot.subsystems.shooter.shooterIndexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterIndexerSubsystem extends SubsystemBase {
  private final ShooterIndexerIO io;
  protected final ShooterIndexerIOInputsAutoLogged inputs = new ShooterIndexerIOInputsAutoLogged();

  public ShooterIndexerSubsystem(ShooterIndexerIO io) {
    this.io = io;
  }

  public void setShooterIndexer(double percentage) {
    io.setPercentage(percentage);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getVoltage() {
    return inputs.appliedVolts;
  }

  public void stop() {
    io.stop();
  }

  public Command runShooterIndexerCommand() {
    return Commands.run(() -> setShooterIndexer(1.0), this).finallyDo((interrupted) -> stop());
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterIndexer", inputs);
  }
}
