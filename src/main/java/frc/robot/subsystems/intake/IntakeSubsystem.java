package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
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
  private double targetPercentage;

  private double stuckTime = 0.0;
  private double CURRENT_CUTOFF_THRSHOLD =
      40; // intake motor will shut off if current exceeds this threshold
  private boolean stuck = false;
  private boolean reversed = false;

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
        new LoggedTunableNumber(
            "Intake/RollerRPM" + (side == IntakeSide.LEFT ? "Left" : "Right"), 10.0);

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

  public double getTargetPercentage() {
    return targetPercentage;
  }

  public double getTargetRPM() {
    return targetRpm.get();
  }

  public void setLowered(boolean lowered) {
    if (lowered) {
      this.stuck = false;
      this.reversed = false;
      this.stuckTime = 0;
    }
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
    targetPercentage = 0;
    io.stop();
  }

  public Command runRollerCommand() {
    return Commands.run(() -> setTargetRPM(targetRpm.get()), this);
  }

  public Command toggleIntakeCommand() {
    return Commands.runOnce(() -> setLowered(!inputs.lowered), this);
  }

  public void runVolts(double volts) {
    io.setVoltage(stuck ? 0 : (reversed ? -volts : volts));
  }

  public void setPercentage(double percentage) {
    targetPercentage = percentage;
    io.setPercentage(stuck ? 0 : (reversed ? -percentage : percentage));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public boolean isStuck() {
    return stuck;
  }

  public boolean isReversed() {
    return reversed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double currTime = Timer.getFPGATimestamp();
    if (Math.abs(getVelocityRPM()) > 0) {
      if (!stuck) {
        if (inputs.supplyCurrentAmps < CURRENT_CUTOFF_THRSHOLD) {
          stuckTime = currTime;
        }
      }

      reversed = currTime - stuckTime > 0.25;

      if (currTime - stuckTime > 0.5) {
        stop();
        stuck = true;
      }
      Logger.recordOutput("Intake Stopped", stuck);
      Logger.recordOutput("Intake Stop Time", currTime - stuckTime);
    }

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
