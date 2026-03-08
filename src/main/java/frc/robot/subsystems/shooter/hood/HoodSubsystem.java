package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HoodSubsystem extends SubsystemBase {
  private final HoodSubsystemIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  private boolean hoodZeroed = false;

  private final LoggedTunableNumber targetAngle = new LoggedTunableNumber("Hood/TargetAngle", 3.15);
  private final LoggedTunableNumber zeroWait = new LoggedTunableNumber("Hood/ZeroWait", 0.3);
  public static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg", 1.0);

  private static final LoggedTunableNumber homingVelocityThreshold =
      new LoggedTunableNumber("Hood/Homing/VelocityThreshold", 0.05);

  private static final LoggedTunableNumber deadbandDeg =
      new LoggedTunableNumber("Hood/DeadbandDeg", 0.5);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", Constants.Subsystems.Shooter.Hood.ClosedLoop.KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Hood/kI", Constants.Subsystems.Shooter.Hood.ClosedLoop.KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", Constants.Subsystems.Shooter.Hood.ClosedLoop.KD);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Hood/kS", Constants.Subsystems.Shooter.Hood.ClosedLoop.KS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Hood/kV", Constants.Subsystems.Shooter.Hood.ClosedLoop.KV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Hood/kA", Constants.Subsystems.Shooter.Hood.ClosedLoop.KA);

  private final LoggedMechanism2d mech =
      new LoggedMechanism2d(0.5, 0.5, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d root = mech.getRoot("HoodPivot", 0.1, 0.1);
  private final LoggedMechanismLigament2d hood =
      root.append(
          new LoggedMechanismLigament2d("Hood", 0.091, 0, 10, new Color8Bit(Color.kOrange)));

  private final LoggedMechanism2d mechSetpoint =
      new LoggedMechanism2d(0.5, 0.5, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d rootSetpoint = mechSetpoint.getRoot("HoodPivot", 0.1, 0.1);
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

    // Apply initial tuning values
    io.configPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (Constants.tuningMode) {
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

    hood.setAngle(inputs.angle);
    Logger.recordOutput("Hood/Mechanism", mech);

    hoodSetpoint.setAngle(inputs.targetAngle);
    Logger.recordOutput("Hood/SetpointMechanism", mechSetpoint);

    var poses = mech.generate3dMechanism();
    var rootPose = new Pose3d(-0.14, 0.0, 0.41, new Rotation3d(0.0, 0.0, Math.PI));

    for (int i = 0; i < poses.size(); i++) {
      poses.set(
          i,
          rootPose.transformBy(
              new Transform3d(poses.get(i).getTranslation(), poses.get(i).getRotation())));
    }
    Logger.recordOutput("Hood/Components/HoodPose3d", poses.toArray(new Pose3d[0]));

    Logger.recordOutput("Hood/HasZeroed", isZeroed());
  }

  public void setTargetAngle(double degrees) {
    if (hoodZeroed) {
      if (Math.abs(getAngle() - degrees) <= deadbandDeg.get()) {
        io.stop();
        return;
      }
      io.setTargetAngle(degrees);
    }
  }

  public void stop() {
    io.stop();
  }

  public double getAngle() {
    return inputs.angle;
  }

  public boolean atSetpoint() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(getAngle() - inputs.targetAngle) <= toleranceDeg.get();
  }

  public Command setTargetAngleCommand(double degrees) {
    return run(() -> setTargetAngle(degrees));
  }

  public Command runTargetAngleCommand() {
    return run(() -> setTargetAngle(targetAngle.get()));
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = LaunchCalculator.getInstance().getParameters();
          io.setTargetAngleWithVelocity(
              Units.radiansToDegrees(params.hoodAngle()), params.hoodVelocity());
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Command to zero the hood. Lowers manually until velocity drops below threshold, then resets
   * position to MIN_ANGLE. Includes a tunable delay to ignore initial inrush.
   */
  public Command zeroCommand() {
    return run(() -> io.setVoltage(Constants.Subsystems.Shooter.Hood.HOMING_VOLTS))
        .raceWith(
            Commands.waitSeconds(zeroWait.get())
                .andThen(
                    Commands.waitUntil(
                        () -> Math.abs(inputs.velocityRPM) <= homingVelocityThreshold.get())))
        .andThen(this::zero);
  }

  public void zero() {
    io.setAngle(Constants.Subsystems.Shooter.Hood.MIN_ANGLE_DEG);
    hoodZeroed = true;
  }

  public boolean isZeroed() {
    return hoodZeroed;
  }
}
