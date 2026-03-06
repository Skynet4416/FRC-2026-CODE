package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakeSubsystemIOTalonFX implements IntakeSubsystemIO {

  private final TalonFX motor;
  private final DoubleSolenoid solenoid;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intake motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;

  public IntakeSubsystemIOTalonFX(IntakeSubsystem.IntakeSide side) {
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

    motor = new TalonFX(motorId);
    solenoid = new DoubleSolenoid(4, PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Intake.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Intake.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Intake.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Intake.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Intake.ClosedLoop.KV;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Intake.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Intake.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Intake.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Intake.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Default
    config.MotorOutput.NeutralMode =
        Constants.Subsystems.Intake.ROLLER_BREAK ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    motor.getConfigurator().apply(config);

    setLowered(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.lowered = (solenoid.get() == DoubleSolenoid.Value.kForward);

    inputs.connected = motorConnectedDebouncer.calculate(motor.isConnected());
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Intake.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLowered(boolean lowered) {
    System.out.println(
        "Setting Soleoind to "
            + lowered
            + " ("
            + solenoid.getFwdChannel()
            + " / "
            + solenoid.getRevChannel()
            + ")");
    solenoid.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.currentSetpoint = 0.0;
  }
}
