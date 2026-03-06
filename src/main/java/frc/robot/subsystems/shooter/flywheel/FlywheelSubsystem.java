package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelSubsystemIO io;
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("FlywheelRPM", Constants.Subsystems.Shooter.Flywheel.DEFAULT_TARGET_RPM);

  protected final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelSubsystemIO io) {
    this.io = io;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public void setTargetRADS(double rads) {
    io.setTargetRADS(rads);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getTempCelsius() {
    return inputs.tempCelsius;
  }

  public void stop() {
    io.stop();
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command runFlywheelCommand() {
    return run(() -> setTargetRPM(targetRpm.get()));
  }

  public Command runFlywheelAtFixedSpeedCommand(double speedRPM) {
    return run(() -> setTargetRPM(speedRPM));
  }

  public Command runAtSpeedRADSCommand(DoubleSupplier speedRADS) {
    return run(() -> setTargetRADS(speedRADS.getAsDouble()));
  }

  public Command runTrackTargetCommand() {
    return run(() -> setTargetRADS(LaunchCalculator.getInstance().getParameters().flywheelSpeed()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
