package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  private final HoodSubsystemIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  private final LoggedTunableNumber targetAngle = new LoggedTunableNumber("Hood/TargetAngle", 0.0);
  private final LoggedTunableNumber zeroWait = new LoggedTunableNumber("Hood/ZeroWait", 0.5);

  public HoodSubsystem(HoodSubsystemIO io) {
    this.io = io;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Hood/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setTargetAngle(double degrees) {
    io.setTargetAngle(degrees);
  }

  public void stop() {
    io.stop();
  }

  public double getAngle() {
    return inputs.angle;
  }

  public Command setTargetAngleCommand(double degrees) {
    return run(() -> setTargetAngle(degrees));
  }

  public Command runTargetAngleCommand() {
    return run(() -> setTargetAngle(targetAngle.get()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Command to zero the hood. Lowers manually until current spike is detected, then resets position
   * to 0. Includes a tunable delay to ignore initial inrush current.
   */
  public Command zeroCommand() {
    return run(() -> io.set(Constants.Subsystems.Shooter.Hood.ZERO_SPEED))
        .raceWith(new frc.robot.util.SuppliedWaitCommand(() -> zeroWait.get()))
        .andThen(
            runEnd(
                    () -> io.set(Constants.Subsystems.Shooter.Hood.ZERO_SPEED),
                    () -> {
                      io.stop();
                      io.setAngle(0.0);
                    })
                .until(
                    () ->
                        Math.abs(inputs.supplyCurrentAmps)
                            > Constants.Subsystems.Shooter.Hood.STALL_CURRENT_LIMIT));
  }
}
