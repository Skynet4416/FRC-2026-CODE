package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HoodSubsystem extends SubsystemBase {
  private final HoodSubsystemIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  private final LoggedTunableNumber targetAngle = new LoggedTunableNumber("Hood/TargetAngle", 0.0);
  private final LoggedTunableNumber zeroWait = new LoggedTunableNumber("Hood/ZeroWait", 0.5);

  private final LoggedMechanism2d mech =
      new LoggedMechanism2d(0.5, 0.5, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d root = mech.getRoot("HoodPivot", 0.1, 0.1);
  private final LoggedMechanismLigament2d hood =
      root.append(
          new LoggedMechanismLigament2d("Hood", 0.091, 0, 10, new Color8Bit(Color.kOrange)));

  private final LoggedMechanism2d mechSetpoint =
      new LoggedMechanism2d(0.5, 0.5, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d rootSetpoint =
      mechSetpoint.getRoot("HoodPivot", 0.1, 0.1);
  private final LoggedMechanismLigament2d hoodSetpoint =
      rootSetpoint.append(
          new LoggedMechanismLigament2d("Hood", 0.091, 0, 10, new Color8Bit(Color.kGreen)));

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

    hood.setAngle(inputs.angle);
    Logger.recordOutput("Hood/Mechanism", mech);

    hoodSetpoint.setAngle(inputs.targetAngle);
    Logger.recordOutput("Hood/SetpointMechanism", mechSetpoint);
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
