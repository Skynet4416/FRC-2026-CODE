package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeSide {
    LEFT,
    RIGHT
  }

  private final IntakeSubsystemIO io;
  private final IntakeSide side;
  private final SysIdRoutine sysIdRoutine;
  private final LoggedTunableNumber targetRpm;
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Mechanism2d
  private final LoggedMechanism2d mech;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d arm;

  public IntakeSubsystem(IntakeSubsystemIO io, IntakeSide side) {
    this.io = io;
    this.side = side;

    String prefix = side == IntakeSide.LEFT ? "IntakeLeft" : "IntakeRight";
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(prefix + "/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    this.targetRpm =
        new LoggedTunableNumber("RollerRPM" + (side == IntakeSide.LEFT ? "Left" : "Right"), 10.0);

    String mechName = side == IntakeSide.LEFT ? "IntakeLeft" : "IntakeRight";
    this.mech = new LoggedMechanism2d(1.0, 1.0, new Color8Bit(Color.kBlack));
    this.root = mech.getRoot(mechName + "Pivot", 0, 0);
    this.arm =
        root.append(
            new LoggedMechanismLigament2d(
                mechName + "Arm", 0.7, 0, 10, new Color8Bit(Color.kOrange)));
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public double getTargetRPM() {
    return targetRpm.get();
  }

  public void setLowered(boolean lowered) {
    System.out.println("SETTING LOEWRED: " + lowered);
    io.setLowered(lowered);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public boolean isLowered() {
    return inputs.lowered;
  }

  public void stop() {
    io.stop();
  }

  public Command runRollerCommand() {
    return Commands.run(() -> setTargetRPM(targetRpm.get()), this);
  }

  public Command toggleIntakeCommand() {
    return Commands.runOnce(() -> setLowered(!inputs.lowered), this);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
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
    String prefix = side == IntakeSide.LEFT ? "IntakeLeft" : "IntakeRight";
    Logger.processInputs(prefix, inputs);

    // Code for displaying intake mechanism in AdvantageScope
    // Update Mechanism2d
    double armAngle = inputs.lowered ? 0.0 : 90.0;

    arm.setAngle(armAngle);

    Logger.recordOutput(prefix + "/Mechanism", mech);

    // Intake Base Pose
    double intakeX = 0.0;
    double intakeY = side == IntakeSide.LEFT ? 0.2 : -0.2;
    double intakeZ = 0.35;
    double intakeYaw = side == IntakeSide.LEFT ? 0.0 : 180.0;

    // Pivot pose
    Pose3d pivotPose =
        new Pose3d(
            new Translation3d(
                intakeX, inputs.lowered ? intakeY : 0.0, inputs.lowered ? intakeZ : 0.0),
            new Rotation3d(0.0, 0.0, Math.toRadians(intakeYaw)));

    // Create a rotation transform based on the state
    double targetX = inputs.lowered ? 0.0 : Math.toRadians(90.0);

    Transform3d armRotation =
        new Transform3d(new Translation3d(), new Rotation3d(targetX, 0.0, 0.0));

    // Apply the transform to the pivot pose
    Pose3d correctLocation = pivotPose.plus(armRotation);
    Logger.recordOutput(prefix + "/Pose", correctLocation);
  }
}
