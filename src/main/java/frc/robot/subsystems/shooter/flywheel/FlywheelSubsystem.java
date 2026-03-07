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
  private final LoggedTunableNumber targetRpm =
      new LoggedTunableNumber(
          "Flywheel/TargetRPM", Constants.Subsystems.Shooter.Flywheel.DEFAULT_TARGET_RPM);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Flywheel/kP", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Flywheel/kI", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Flywheel/kD", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KD);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Flywheel/kS", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Flywheel/kV", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Flywheel/kA", Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KA);

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

    // Handle initial tuning values
    io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }
}
