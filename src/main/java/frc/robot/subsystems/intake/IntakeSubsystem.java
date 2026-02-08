package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeSubsystemIO io;
  private final LoggedTunableNumber targetRpm = new LoggedTunableNumber("RollerRPM", 500.0);
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Mechanism2d
  private final LoggedMechanism2d mech =
      new LoggedMechanism2d(1.0, 1.0, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d root = mech.getRoot("IntakePivot", 0, 0);
  private final LoggedMechanismLigament2d arm =
      root.append(
          new LoggedMechanismLigament2d("IntakeArm", 0.7, 0, 10, new Color8Bit(Color.kOrange)));

  public IntakeSubsystem(IntakeSubsystemIO io) {
    this.io = io;
  }

  public void setTargetRPM(double rpm) {
    io.setTargetRPM(rpm);
  }

  public void setLowered(boolean lowered) {
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
    io.setVoltage(0);
  }

  public Command runRollerCommand() {
    return Commands.runEnd(() -> setTargetRPM(targetRpm.get()), () -> stop(), this);
  }

  public Command toggleIntakeCommand() {
    return Commands.runOnce(() -> setLowered(!inputs.lowered), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Code for displaying intake mechanism in AdvantageScope
    // Update Mechanism2d
    double armAngle = inputs.lowered ? 0.0 : 90.0;

    arm.setAngle(armAngle);

    Logger.recordOutput("Intake/Mechanism", mech);
    Logger.recordOutput(
        "Intake/Components/Mech3d", mech.generate3dMechanism().toArray(new Pose3d[0]));

    // Intake1 Base Pose
    double intake1X = 0.0;
    double intake1Y = 0.2;
    double intake1Z = 0.35;
    double intake1Yaw = 0.0;

    // Intake 1
    // Manual correct location pose calculation
    // Start with the pivot pose the tunable location
    Pose3d pivotPose =
        new Pose3d(
            new Translation3d(
                intake1X, inputs.lowered ? intake1Y : 0.0, inputs.lowered ? intake1Z : 0.0),
            new Rotation3d(0.0, 0.0, Math.toRadians(intake1Yaw))); // Original orientation

    // Create a rotation transform based on the state
    // Lowered (true) -> 0 degrees
    // Raised (false) -> 90 degrees (Up via Roll)
    double targetX = inputs.lowered ? 0.0 : Math.toRadians(90.0);

    Transform3d armRotation =
        new Transform3d(new Translation3d(), new Rotation3d(targetX, 0.0, 0.0));

    // Apply the transform to the pivot pose
    Pose3d correctLocation = pivotPose.plus(armRotation);
    Logger.recordOutput("Intake/IntakePose1", correctLocation);

    // Intake 2
    // Mirrored from Intake 1
    double setpoint2XVal = intake1X;
    double setpoint2YVal = -intake1Y; // Mirror Y
    double setpoint2ZVal = intake1Z;
    double setpoint2YawVal = intake1Yaw + 180.0; // Rotate 180

    // Same position but rotated 180 degrees Yaw
    Pose3d pivotPose2 =
        new Pose3d(
            new Translation3d(
                setpoint2XVal,
                inputs.lowered ? setpoint2YVal : 0.0,
                inputs.lowered ? setpoint2ZVal : 0.0),
            new Rotation3d(0.0, 0.0, Math.toRadians(setpoint2YawVal)));

    // Use same targetX because both fold "Up" in their local frame
    double targetX2 = inputs.lowered ? 0.0 : Math.toRadians(90.0);

    Transform3d armRotation2 =
        new Transform3d(new Translation3d(), new Rotation3d(targetX2, 0.0, 0.0));

    Pose3d correctLocation2 = pivotPose2.plus(armRotation2);

    Logger.recordOutput("Intake/IntakePose2", correctLocation2);
  }
}
