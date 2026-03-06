package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakeSubsystemIOSparkMax implements IntakeSubsystemIO {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final DoubleSolenoid solenoid;
  private final ClosedLoopConfig closedLoopConfig;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intake motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;

  public IntakeSubsystemIOSparkMax(IntakeSubsystem.IntakeSide side) {
    int motorId =
        side == IntakeSubsystem.IntakeSide.LEFT
            ? Constants.Subsystems.Intake.Id.Motor.LEFT_ROLLER
            : Constants.Subsystems.Intake.Id.Motor.RIGHT_ROLLER;
    int forwardChannel =
        side == IntakeSubsystem.IntakeSide.LEFT
            ? Constants.Subsystems.Intake.Id.Pneumatics.LEFT_FORWARDS
            : Constants.Subsystems.Intake.Id.Pneumatics.RIGHT_FORWARDS;
    int reverseChannel =
        side == IntakeSubsystem.IntakeSide.LEFT
            ? Constants.Subsystems.Intake.Id.Pneumatics.LEFT_REVERSE
            : Constants.Subsystems.Intake.Id.Pneumatics.RIGHT_REVERSE;

    this.motor = new SparkMax(motorId, MotorType.kBrushless);
    this.solenoid =
        new DoubleSolenoid(4, PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    this.closedLoopConfig =
        new ClosedLoopConfig()
            .p(Constants.Subsystems.Intake.ClosedLoop.KP)
            .i(Constants.Subsystems.Intake.ClosedLoop.KI)
            .d(Constants.Subsystems.Intake.ClosedLoop.KD)
            .apply(
                new FeedForwardConfig()
                    .kV(Constants.Subsystems.Intake.ClosedLoop.KV)
                    .kS(Constants.Subsystems.Intake.ClosedLoop.KS));
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystems.Intake.ROLLER_BREAK
                ? SparkBaseConfig.IdleMode.kBrake
                : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .apply(closedLoopConfig);

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.setLowered(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = this.motor.getEncoder().getVelocity();
    inputs.appliedVolts = this.motor.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.supplyCurrentAmps = this.motor.getOutputCurrent();
    inputs.lowered = (this.solenoid.get() == DoubleSolenoid.Value.kForward);

    inputs.connected = motorConnectedDebouncer.calculate(this.motor.getFirmwareVersion() != 0);
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Intake.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    this.motor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
  }

  @Override
  public void setLowered(boolean lowered) {
    this.solenoid.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.currentSetpoint = 0.0;
  }
}
